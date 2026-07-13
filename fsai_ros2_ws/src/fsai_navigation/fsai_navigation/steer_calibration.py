"""Open-loop steering calibration for the CarMaker (CMRosIF) simulated vehicle.

Purpose
───────
The navigation controllers publish ``VehicleControl.steer_ang`` as a *desired
road-wheel angle in radians* (bicycle model, δ = atan(L·κ)).  Whether the
simulated plant actually realises that angle 1:1 — or attenuates / rate-limits /
lags it — is a property of the CarMaker vehicle + CMRosIF bridge, NOT of the
navigation stack.  This node measures that plant response directly so the
sim-specific compensation can live in a thin, swappable calibration layer
instead of being smeared into controller gains.

Method
──────
Drives the car open-loop at a roughly constant low speed and steps
``steer_ang`` through a sequence of constant setpoints, holding each for a few
seconds.  From ``/carmaker/odom`` it computes the achieved path curvature

    κ_achieved = yaw_rate / forward_speed         (kinematic, speed-independent)
    δ_achieved = atan(wheelbase · κ_achieved)     (implied road-wheel angle)

For each setpoint it records the steady-state mean and the step rise-time, then
fits a line δ_achieved ≈ gain · steer_ang_commanded over the near-linear region.
Results are written to a YAML report; ``gain`` (and its inverse — the multiplier
a controller would apply to a desired δ to get the sim to execute it) are the
headline numbers.

This is a SIM calibration.  On the real vehicle the mapping is expected to be
identity and this file should not be used.

Run standalone (NOT alongside a path follower — they would fight for the control
topic).  Requires CarMaker + the CMRosIF bridge running and the car in open
space:

    ros2 run fsai_navigation steer_calibration \
        --ros-args -p output_path:=/tmp/steer_calibration.yaml
"""

from __future__ import annotations

import enum
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from vehiclecontrol_msgs.msg import VehicleControl


class _Phase(enum.Enum):
    WARMUP = 'warmup'      # accelerate to target speed with steer=0
    STEP = 'step'          # hold a constant steer_ang setpoint
    DONE = 'done'          # finished: brake to a stop


class SteerCalibration(Node):
    def __init__(self) -> None:
        super().__init__('steer_calibration')

        self.declare_parameter('control_topic', '/carmaker/VehicleControl')
        self.declare_parameter('odom_topic', '/carmaker/odom')
        self.declare_parameter('output_path', '/tmp/steer_calibration.yaml')
        self.declare_parameter('wheelbase', 1.53)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('selector_ctrl', 1)

        # Speed control (simple P on gas + light brake) to keep κ = yaw/v clean.
        self.declare_parameter('target_speed_mps', 1.0)
        self.declare_parameter('gas_kp', 0.10)
        self.declare_parameter('gas_ff', 0.03)
        self.declare_parameter('max_gas', 0.15)
        self.declare_parameter('brake_kp', 0.30)
        self.declare_parameter('max_brake', 0.30)
        self.declare_parameter('min_speed_mps', 0.3)   # below this, κ samples rejected

        # Step schedule.
        self.declare_parameter('warmup_sec', 4.0)
        self.declare_parameter('hold_sec', 5.0)        # per setpoint
        self.declare_parameter('settle_frac', 0.5)     # last fraction used for steady mean
        # Commanded road-wheel-angle setpoints (rad). Ascending +, then -.
        self.declare_parameter(
            'steer_setpoints',
            [0.05, 0.10, 0.15, 0.20, 0.30, 0.40,
             -0.05, -0.10, -0.15, -0.20, -0.30, -0.40],
        )
        # Region used for the linear gain fit.
        self.declare_parameter('linear_fit_max_abs', 0.40)

        self._control_topic = str(self.get_parameter('control_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        self._output_path = str(self.get_parameter('output_path').value)
        self._wheelbase = float(self.get_parameter('wheelbase').value)
        rate = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self._selector = int(self.get_parameter('selector_ctrl').value)

        self._v_target = float(self.get_parameter('target_speed_mps').value)
        self._gas_kp = float(self.get_parameter('gas_kp').value)
        self._gas_ff = float(self.get_parameter('gas_ff').value)
        self._max_gas = float(self.get_parameter('max_gas').value)
        self._brake_kp = float(self.get_parameter('brake_kp').value)
        self._max_brake = float(self.get_parameter('max_brake').value)
        self._min_speed = float(self.get_parameter('min_speed_mps').value)

        self._warmup_sec = float(self.get_parameter('warmup_sec').value)
        self._hold_sec = float(self.get_parameter('hold_sec').value)
        self._settle_frac = min(0.9, max(0.1, float(self.get_parameter('settle_frac').value)))
        self._setpoints = [float(s) for s in self.get_parameter('steer_setpoints').value]
        self._linear_fit_max = float(self.get_parameter('linear_fit_max_abs').value)

        # Live odom state.
        self._speed: float | None = None
        self._yaw_rate: float | None = None

        # Sequencing.
        self._phase = _Phase.WARMUP
        self._phase_start = self._now()
        self._step_index = 0
        # Per-hold sample buffer: list of (t, kappa) with speed >= min_speed.
        self._buf: list[tuple[float, float]] = []
        self._records: list[dict] = []
        self._written = False

        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._pub = self.create_publisher(VehicleControl, self._control_topic, 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'Steer calibration ready  setpoints={self._setpoints}  '
            f'v_target={self._v_target:.2f} m/s  hold={self._hold_sec:.1f}s  '
            f'output={self._output_path}\n'
            f'  Run this WITHOUT a path follower active. Car will drive in circles.'
        )

    # ── Odom ────────────────────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry) -> None:
        self._speed = msg.twist.twist.linear.x
        self._yaw_rate = msg.twist.twist.angular.z
        if self._phase == _Phase.STEP and self._speed is not None:
            if self._speed >= self._min_speed:
                kappa = self._yaw_rate / self._speed
                self._buf.append((self._now(), kappa))

    # ── Control loop ──────────────────────────────────────────────────────────

    def _tick(self) -> None:
        elapsed = self._now() - self._phase_start

        if self._phase == _Phase.WARMUP:
            self._publish(steer=0.0, drive=True)
            if elapsed >= self._warmup_sec:
                self._start_step(0)
            return

        if self._phase == _Phase.STEP:
            self._publish(steer=self._setpoints[self._step_index], drive=True)
            if elapsed >= self._hold_sec:
                self._finish_step()
            return

        # DONE
        self._publish(steer=0.0, drive=False)  # brake to a stop
        if not self._written:
            self._write_report()
            self._written = True

    def _start_step(self, index: int) -> None:
        self._step_index = index
        self._phase = _Phase.STEP
        self._phase_start = self._now()
        self._buf = []
        self.get_logger().info(
            f'[step {index + 1}/{len(self._setpoints)}] '
            f'steer_ang={self._setpoints[index]:+.3f} rad — holding {self._hold_sec:.1f}s'
        )

    def _finish_step(self) -> None:
        setpoint = self._setpoints[self._step_index]
        record = self._summarise_hold(setpoint, self._buf)
        self._records.append(record)
        self.get_logger().info(
            f'  → cmd={setpoint:+.3f}  achieved κ={record["achieved_curvature"]:+.4f}  '
            f'roadwheel δ={record["achieved_roadwheel"]:+.4f} rad  '
            f'v={record["mean_speed"]:.2f}  rise={record["rise_time_sec"]:.2f}s  '
            f'n={record["n_samples"]}'
        )
        nxt = self._step_index + 1
        if nxt < len(self._setpoints):
            self._start_step(nxt)
        else:
            self._phase = _Phase.DONE
            self._phase_start = self._now()

    def _summarise_hold(self, setpoint: float, buf: list[tuple[float, float]]) -> dict:
        if not buf:
            return {
                'commanded_steer_ang': setpoint,
                'achieved_curvature': float('nan'),
                'achieved_roadwheel': float('nan'),
                'mean_speed': float('nan'),
                'rise_time_sec': float('nan'),
                'n_samples': 0,
            }
        t0 = buf[0][0]
        t_end = buf[-1][0]
        span = max(1e-3, t_end - t0)
        settle_start = t_end - self._settle_frac * span
        steady = [k for (t, k) in buf if t >= settle_start]
        if not steady:
            steady = [k for (_, k) in buf]
        mean_kappa = sum(steady) / len(steady)
        roadwheel = math.atan(self._wheelbase * mean_kappa)

        # Rise time: first crossing of 63% of the steady curvature magnitude.
        rise = float('nan')
        target = 0.63 * abs(mean_kappa)
        for t, k in buf:
            if abs(k) >= target:
                rise = t - t0
                break

        # Mean speed over the hold (needs raw speed; approximate from availability).
        mean_speed = self._speed if self._speed is not None else float('nan')
        return {
            'commanded_steer_ang': setpoint,
            'achieved_curvature': mean_kappa,
            'achieved_roadwheel': roadwheel,
            'mean_speed': mean_speed,
            'rise_time_sec': rise,
            'n_samples': len(buf),
        }

    # ── Fit + report ──────────────────────────────────────────────────────────

    def _linear_gain(self) -> tuple[float, int]:
        """Least-squares slope of achieved_roadwheel vs commanded_steer_ang
        (through origin) over |commanded| <= linear_fit_max. Returns (gain, n)."""
        num = 0.0
        den = 0.0
        n = 0
        for r in self._records:
            x = r['commanded_steer_ang']
            y = r['achieved_roadwheel']
            if math.isnan(y) or abs(x) > self._linear_fit_max + 1e-9:
                continue
            num += x * y
            den += x * x
            n += 1
        gain = num / den if den > 1e-9 else float('nan')
        return gain, n

    def _write_report(self) -> None:
        gain, n_fit = self._linear_gain()
        inv = (1.0 / gain) if (not math.isnan(gain) and abs(gain) > 1e-6) else float('nan')
        lines: list[str] = []
        lines.append('# Steering calibration — CarMaker (CMRosIF) sim plant')
        lines.append('# steer_ang is commanded desired road-wheel angle [rad];')
        lines.append('# achieved_roadwheel is atan(wheelbase * yaw_rate/speed) [rad].')
        lines.append(f'wheelbase: {self._wheelbase:.4f}')
        lines.append(f'target_speed_mps: {self._v_target:.3f}')
        lines.append(f'linear_fit_max_abs: {self._linear_fit_max:.3f}')
        lines.append(f'fit_points: {n_fit}')
        lines.append(f'achieved_per_commanded_gain: {gain:.4f}')
        lines.append(f'publish_multiplier_to_achieve_desired: {inv:.4f}')
        lines.append('records:')
        for r in self._records:
            lines.append(
                f'  - commanded_steer_ang: {r["commanded_steer_ang"]:.4f}\n'
                f'    achieved_curvature: {r["achieved_curvature"]:.6f}\n'
                f'    achieved_roadwheel: {r["achieved_roadwheel"]:.6f}\n'
                f'    mean_speed: {r["mean_speed"]:.4f}\n'
                f'    rise_time_sec: {r["rise_time_sec"]:.4f}\n'
                f'    n_samples: {r["n_samples"]}'
            )
        text = '\n'.join(lines) + '\n'
        try:
            with open(self._output_path, 'w') as f:
                f.write(text)
            self.get_logger().info(
                f'Calibration written to {self._output_path}\n'
                f'  achieved/commanded gain = {gain:.4f}  '
                f'(publish x{inv:.3f} to hit a desired road-wheel angle)'
            )
        except OSError as exc:
            self.get_logger().error(f'Failed to write {self._output_path}: {exc}')
            self.get_logger().info('Calibration report:\n' + text)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _speed_command(self) -> tuple[float, float]:
        if self._speed is None:
            return self._gas_ff, 0.0
        err = self._v_target - self._speed
        gas = self._clamp(self._gas_ff + self._gas_kp * err, 0.0, self._max_gas)
        brake = 0.0
        if self._speed > self._v_target:
            gas = 0.0
            brake = self._clamp(self._brake_kp * (self._speed - self._v_target),
                                0.0, self._max_brake)
        return gas, brake

    def _publish(self, steer: float, drive: bool) -> None:
        msg = VehicleControl()
        msg.use_vc = True
        msg.selector_ctrl = self._selector
        if drive:
            gas, brake = self._speed_command()
        else:
            gas, brake = 0.0, self._max_brake
        msg.gas = float(self._clamp(gas, 0.0, 1.0))
        msg.brake = float(self._clamp(brake, 0.0, 1.0))
        msg.steer_ang = float(steer)
        msg.steer_ang_vel = 0.0
        msg.steer_ang_acc = 0.0
        self._pub.publish(msg)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _clamp(v: float, lo: float, hi: float) -> float:
        return max(lo, min(v, hi))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SteerCalibration()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Best-effort safe stop.
        try:
            node._publish(steer=0.0, drive=False)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
