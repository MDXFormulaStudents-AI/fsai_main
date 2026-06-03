"""Executes YAML-defined static inspection drive profiles."""

from __future__ import annotations

from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from fsai_interfaces.msg import DriveCommand, InterfaceState, WheelSpeeds
from std_msgs.msg import Bool, String


COMMAND_FIELDS = ('steer_angle_deg', 'axle_speed_rpm', 'axle_torque_nm', 'brake_pct')


class StaticProfileExecutor(Node):
    """Publishes static inspection candidate commands for the mission manager."""

    def __init__(self) -> None:
        super().__init__('static_profile_executor')

        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('profile_directory', '')
        self.declare_parameter('wheel_circumference_m', 1.674)

        publish_rate_hz = max(
            1.0,
            self.get_parameter('publish_rate_hz').get_parameter_value().double_value,
        )
        profile_directory = self.get_parameter('profile_directory').get_parameter_value().string_value
        self._wheel_circumference_m = (
            self.get_parameter('wheel_circumference_m').get_parameter_value().double_value
        )

        if profile_directory:
            self._profile_dir = Path(profile_directory)
        else:
            package_share = Path(get_package_share_directory('fsai_mission_control'))
            self._profile_dir = package_share / 'profiles'

        self._mission = 'none'
        self._profile: dict | None = None
        self._profile_path: Path | None = None
        self._interface_state = InterfaceState.WAIT_FOR_VCU
        self._wheel_speeds: WheelSpeeds | None = None
        self._completed = False
        self._last_command: dict[str, float] | None = None
        self._waiting_for_stop_logged = False

        # Per-step tracking — supports both duration_s and distance_m step types
        self._start_sec: float | None = None
        self._current_step_idx: int = 0
        self._step_start_sec: float | None = None
        self._step_start_distance_m: float = 0.0
        self._total_distance_m: float = 0.0
        self._last_odometry_sec: float | None = None
        self._last_distance_log_sec: float | None = None

        self.create_subscription(String, '/mission/selected', self._on_mission_selected, 10)
        self.create_subscription(
            InterfaceState,
            '/vehicle/interface_state',
            self._on_interface_state,
            10,
        )
        self.create_subscription(WheelSpeeds, '/vcu/wheel_speeds', self._on_wheel_speeds, 10)

        self._drive_pub = self.create_publisher(DriveCommand, '/static_drive_command', 10)
        self._mission_complete_pub = self.create_publisher(Bool, '/static_mission_complete', 10)
        self._estop_pub = self.create_publisher(Bool, '/static_estop', 10)
        self.create_timer(1.0 / publish_rate_hz, self._tick)

        profile_dir_display = f'.../{self._profile_dir.parent.name}/{self._profile_dir.name}'
        self.get_logger().info(f'── Static Profile Executor ready ── profiles: {profile_dir_display}')

    # ── Subscriptions ─────────────────────────────────────────────────────────

    def _on_mission_selected(self, msg: String) -> None:
        mission = msg.data.strip()
        if mission == self._mission:
            return

        self._reset_profile_state()
        self._mission = mission
        if mission == 'static_inspection_a':
            self._load_profile('static_inspection_a.yaml')
        elif mission == 'static_inspection_b':
            self._load_profile('static_inspection_b.yaml')
        elif mission == 'autonomous_demo':
            self._load_profile('autonomous_demo.yaml')
        else:
            self._profile = None
            self._profile_path = None

    def _on_interface_state(self, msg: InterfaceState) -> None:
        self._interface_state = int(msg.state)

    def _on_wheel_speeds(self, msg: WheelSpeeds) -> None:
        self._wheel_speeds = msg

    # ── Main tick ─────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        if self._profile is None or self._completed:
            return
        if self._interface_state != InterfaceState.DRIVING:
            if self._start_sec is not None:
                self.get_logger().warn('Profile PAUSED -- interface left DRIVING. Will restart from beginning on re-entry.')
            self._reset_step_state()
            return

        now_sec = self._now_sec()

        if self._start_sec is None:
            self._start_sec = now_sec
            self._step_start_sec = now_sec
            self._step_start_distance_m = 0.0
            self._total_distance_m = 0.0
            self._last_odometry_sec = now_sec
            self._current_step_idx = 0
            profile_name = self._profile_path.name if self._profile_path else '<unknown>'
            self.get_logger().info(f'-------- Profile START: {profile_name} --------')

        self._update_odometry(now_sec)

        steps = self._profile['steps']
        if self._current_step_idx >= len(steps):
            self._complete_profile()
            return

        step = steps[self._current_step_idx]
        elapsed_in_step = now_sec - self._step_start_sec
        distance_in_step = self._total_distance_m - self._step_start_distance_m

        if self._step_complete(step, elapsed_in_step, distance_in_step):
            step_name = step.get('name', f'step_{self._current_step_idx}')
            step_num = self._current_step_idx + 1
            total_steps = len(steps)
            step_type = str(step.get('type', 'hold')).lower()
            detail = (
                f'{distance_in_step:.2f}m in {elapsed_in_step:.2f}s'
                if step_type == 'distance'
                else f'{elapsed_in_step:.2f}s'
            )
            self.get_logger().info(f'[{step_num}/{total_steps}] {step_name}  done  ({detail})')
            self._current_step_idx += 1
            self._step_start_sec = now_sec
            self._step_start_distance_m = self._total_distance_m
            if self._current_step_idx >= len(steps):
                self._complete_profile()
                return
            step = steps[self._current_step_idx]
            elapsed_in_step = 0.0
            self._last_distance_log_sec = None  # log immediately on entering new step

        # Throttled progress log for distance steps (once per second)
        if str(step.get('type', 'hold')).lower() == 'distance':
            if (self._last_distance_log_sec is None or
                    now_sec - self._last_distance_log_sec >= 1.0):
                target_m = float(step.get('distance_m', 0.0))
                step_name = step.get('name', f'step_{self._current_step_idx}')
                wheels_ok = self._wheel_speeds is not None
                self.get_logger().info(
                    f'[distance] {step_name}: {distance_in_step:.2f} / {target_m:.1f} m'
                    f'  (wheel data: {"ok" if wheels_ok else "MISSING"})'
                )
                self._last_distance_log_sec = now_sec

        command = self._command_for_step(step, elapsed_in_step)
        self._last_command = command
        self._publish_drive_command(command)

    # ── Step logic ────────────────────────────────────────────────────────────

    def _step_complete(
        self, step: dict, elapsed_in_step: float, distance_in_step: float
    ) -> bool:
        step_type = str(step.get('type', 'hold')).lower()
        if step_type == 'distance':
            return distance_in_step >= float(step.get('distance_m', 0.0))
        return elapsed_in_step >= max(0.0, float(step.get('duration_s', 0.0)))

    def _command_for_step(self, step: dict, elapsed_in_step: float) -> dict[str, float]:
        step_type = str(step.get('type', 'hold')).lower()
        if step_type == 'ramp':
            duration = max(1e-6, float(step.get('duration_s', 1.0)))
            ratio = self._clamp(elapsed_in_step / duration, 0.0, 1.0)
            start = self._command_dict(step.get('start', {}))
            end = self._command_dict(step.get('end', {}))
            return {
                field: start[field] + ratio * (end[field] - start[field])
                for field in COMMAND_FIELDS
            }
        return self._command_dict(step.get('command', {}))

    # ── Odometry ──────────────────────────────────────────────────────────────

    def _update_odometry(self, now_sec: float) -> None:
        if self._last_odometry_sec is None:
            self._last_odometry_sec = now_sec
            return
        if self._wheel_speeds is None:
            return
        dt = now_sec - self._last_odometry_sec
        self._last_odometry_sec = now_sec
        avg_rpm = (abs(self._wheel_speeds.rl_rpm) + abs(self._wheel_speeds.rr_rpm)) / 2.0
        self._total_distance_m += (avg_rpm / 60.0) * self._wheel_circumference_m * dt

    # ── Profile loading and completion ────────────────────────────────────────

    def _load_profile(self, filename: str) -> None:
        path = self._profile_dir / filename
        with path.open('r', encoding='utf-8') as handle:
            profile = yaml.safe_load(handle) or {}

        if not isinstance(profile.get('steps'), list) or not profile['steps']:
            raise ValueError(f'Profile {path} must define a non-empty steps list')

        self._profile = profile
        self._profile_path = path
        self.get_logger().info(f'Profile loaded: {path.name}')

    def _reset_profile_state(self) -> None:
        self._reset_step_state()
        self._completed = False
        self._last_command = None
        self._waiting_for_stop_logged = False

    def _reset_step_state(self) -> None:
        self._start_sec = None
        self._current_step_idx = 0
        self._step_start_sec = None
        self._step_start_distance_m = 0.0
        self._total_distance_m = 0.0
        self._last_odometry_sec = None
        self._last_distance_log_sec = None

    def _complete_profile(self) -> None:
        assert self._profile is not None

        action = str(self._profile.get('completion_action', 'mission_complete')).lower()
        if action == 'mission_complete' and self._must_wait_for_stop():
            if self._last_command is not None:
                self._publish_drive_command(self._last_command)
            if not self._waiting_for_stop_logged:
                threshold = float(self._profile.get('stopped_rpm_threshold', 10.0))
                self.get_logger().info(f'All steps done -- waiting for wheels to stop (< {threshold:.0f} rpm)...')
                self._waiting_for_stop_logged = True
            return

        if action == 'estop':
            self.get_logger().warn('!! Profile COMPLETE --> SOFTWARE E-STOP REQUESTED !!')
            self._estop_pub.publish(Bool(data=True))
        else:
            self.get_logger().info('-------- Profile COMPLETE --> mission complete signal sent --------')
            self._mission_complete_pub.publish(Bool(data=True))

        self._completed = True

    def _must_wait_for_stop(self) -> bool:
        assert self._profile is not None
        if not bool(self._profile.get('require_stopped_before_completion', False)):
            return False
        if self._wheel_speeds is None:
            return True

        threshold = float(self._profile.get('stopped_rpm_threshold', 10.0))
        speeds = [
            self._wheel_speeds.fl_rpm,
            self._wheel_speeds.fr_rpm,
            self._wheel_speeds.rl_rpm,
            self._wheel_speeds.rr_rpm,
        ]
        return any(abs(speed) > threshold for speed in speeds)

    # ── Publishing ────────────────────────────────────────────────────────────

    def _publish_drive_command(self, command: dict[str, float]) -> None:
        msg = DriveCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.steer_angle_deg = float(command['steer_angle_deg'])
        msg.axle_speed_rpm = max(0.0, float(command['axle_speed_rpm']))
        msg.axle_torque_nm = max(0.0, float(command['axle_torque_nm']))
        msg.brake_pct = self._clamp(float(command['brake_pct']), 0.0, 100.0)
        if msg.brake_pct > 0.0:
            msg.axle_torque_nm = 0.0
        self._drive_pub.publish(msg)

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _command_dict(raw: dict) -> dict[str, float]:
        return {field: float(raw.get(field, 0.0)) for field in COMMAND_FIELDS}

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(value, upper))

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StaticProfileExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == '__main__':
    main()
