#include "fsai_vehicle_interface/vehicle_interface_node.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace fsai_vehicle_interface
{

// ── Constructor ───────────────────────────────────────────────────────────────

VehicleInterfaceNode::VehicleInterfaceNode(const rclcpp::NodeOptions & options)
: Node("vehicle_interface_node", options)
{
  // ── Parameters ─────────────────────────────────────────────────────────────
  this->declare_parameter("can_interface",           "can0");
  this->declare_parameter("stale_command_timeout_ms", 100);
  this->declare_parameter("loop_rate_hz",             100);
  this->declare_parameter("imu_frame_id",             "imu");
  this->declare_parameter("gps_frame_id",             "gps");

  can_interface_             = this->get_parameter("can_interface").as_string();
  stale_command_timeout_ms_  = this->get_parameter("stale_command_timeout_ms").as_int();
  const int loop_rate_hz     = this->get_parameter("loop_rate_hz").as_int();
  imu_frame_id_              = this->get_parameter("imu_frame_id").as_string();
  gps_frame_id_              = this->get_parameter("gps_frame_id").as_string();

  RCLCPP_INFO(this->get_logger(),
    "Parameters: can_interface=%s, loop_rate=%dHz, stale_timeout=%dms",
    can_interface_.c_str(), loop_rate_hz, stale_command_timeout_ms_);

  // ── Publishers ─────────────────────────────────────────────────────────────
  pub_vcu_status_ = this->create_publisher<fsai_interfaces::msg::VcuStatus>(
    "/vcu/status", 10);
  pub_wheel_speeds_ = this->create_publisher<fsai_interfaces::msg::WheelSpeeds>(
    "/vcu/wheel_speeds", 10);
  pub_steering_ = this->create_publisher<std_msgs::msg::Float32>(
    "/vcu/steering", 10);
  pub_brake_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/vcu/brake", 10);
  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "/vcu/imu", 10);
  pub_gps_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
    "/vcu/gps", 10);
  pub_interface_state_ = this->create_publisher<fsai_interfaces::msg::InterfaceState>(
    "/vehicle/interface_state", 10);

  // ── Subscribers ────────────────────────────────────────────────────────────
  sub_drive_command_ = this->create_subscription<fsai_interfaces::msg::DriveCommand>(
    "/vehicle/drive_command", 10,
    std::bind(&VehicleInterfaceNode::drive_command_callback, this, std::placeholders::_1));

  sub_mission_complete_ = this->create_subscription<std_msgs::msg::Bool>(
    "/vehicle/mission_complete", 10,
    std::bind(&VehicleInterfaceNode::mission_complete_callback, this, std::placeholders::_1));

  sub_estop_ = this->create_subscription<std_msgs::msg::Bool>(
    "/vehicle/estop", 1,
    std::bind(&VehicleInterfaceNode::estop_callback, this, std::placeholders::_1));

  // ── Initialise FS-AI API ───────────────────────────────────────────────────
  RCLCPP_INFO(this->get_logger(),
    "Initialising FS-AI API on CAN interface: %s", can_interface_.c_str());

  // debug=0 (silent), simulate=0 (real hardware)
  if (fs_ai_api_init(const_cast<char *>(can_interface_.c_str()), 0, 0) != 0) {
    RCLCPP_FATAL(this->get_logger(),
      "fs_ai_api_init() failed on '%s'. "
      "Check the CAN interface is up: 'ip link show %s'",
      can_interface_.c_str(), can_interface_.c_str());
    throw std::runtime_error("FS-AI API initialisation failed");
  }

  api_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "FS-AI API initialised successfully");

  // Initialise last command time so stale detection works from startup
  last_drive_command_time_ = this->now();

  // ── Main loop timer ────────────────────────────────────────────────────────
  const auto period = std::chrono::milliseconds(1000 / loop_rate_hz);
  timer_ = this->create_wall_timer(
    period,
    std::bind(&VehicleInterfaceNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(),
    "Vehicle interface node ready. Waiting for VCU...");
}

// ── Destructor ────────────────────────────────────────────────────────────────

VehicleInterfaceNode::~VehicleInterfaceNode()
{
  // Send 20 zero frames (~200ms) before shutting down so the VCU
  // does not trigger AI_COMMS_LOST immediately on node exit.
  if (api_initialized_) {
    RCLCPP_INFO(this->get_logger(), "Sending safe shutdown frames...");
    send_zero_frames();
  }
}

// ── Main loop ─────────────────────────────────────────────────────────────────

void VehicleInterfaceNode::timer_callback()
{
  if (!api_initialized_) {
    return;
  }

  // ── 1. Read all data from the API ─────────────────────────────────────────
  fs_ai_api_vcu2ai vcu2ai_data = {};
  fs_ai_api_imu    imu_data    = {};
  fs_ai_api_gps    gps_data    = {};
  can_stats_t      can_stats   = {};

  fs_ai_api_vcu2ai_get_data(&vcu2ai_data);
  fs_ai_api_imu_get_data(&imu_data);
  fs_ai_api_gps_get_data(&gps_data);
  fs_ai_api_get_can_stats(&can_stats);

  // Detect first VCU status frame using the CAN stats counter
  if (!has_vcu_status_ && can_stats.VCU2AI_Status_count > 0) {
    has_vcu_status_ = true;
    RCLCPP_INFO(this->get_logger(), "First VCU2AI_Status frame received — VCU is alive");
  }

  // ── 2. Publish all VCU data ───────────────────────────────────────────────
  publish_vcu_status(vcu2ai_data);
  publish_wheel_speeds(vcu2ai_data);
  publish_steering(vcu2ai_data);
  publish_brake(vcu2ai_data);
  publish_imu(imu_data);
  publish_gps(gps_data);

  // ── 3. Update state machine ───────────────────────────────────────────────
  state_machine_.update(vcu2ai_data, mission_complete_received_, has_vcu_status_);
  mission_complete_received_ = false;  // consume the signal

  // ── 4. Publish interface state ────────────────────────────────────────────
  publish_interface_state();

  // ── 5. Build and send AI2VCU command ─────────────────────────────────────
  fs_ai_api_ai2vcu ai2vcu = {};

  // Handshake: mirror what the VCU sends
  ai2vcu.AI2VCU_HANDSHAKE_SEND_BIT =
    (vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT == HANDSHAKE_RECEIVE_BIT_ON)
    ? HANDSHAKE_SEND_BIT_ON
    : HANDSHAKE_SEND_BIT_OFF;

  // State machine owns mission status and direction
  ai2vcu.AI2VCU_MISSION_STATUS    = state_machine_.get_mission_status();
  ai2vcu.AI2VCU_DIRECTION_REQUEST = state_machine_.get_direction();

  // E-stop: physical RES always takes priority; software can also latch it via /vehicle/estop
  ai2vcu.AI2VCU_ESTOP_REQUEST = estop_requested_ ? ESTOP_YES : ESTOP_NO;

  // Drive commands: only forward in DRIVING state with a fresh command
  if (state_machine_.should_forward_commands() && is_drive_command_fresh() && latest_drive_command_) {
    ai2vcu.AI2VCU_STEER_ANGLE_REQUEST_deg = latest_drive_command_->steer_angle_deg;
    ai2vcu.AI2VCU_AXLE_SPEED_REQUEST_rpm  = latest_drive_command_->axle_speed_rpm;
    ai2vcu.AI2VCU_AXLE_TORQUE_REQUEST_Nm  = latest_drive_command_->axle_torque_nm;
    ai2vcu.AI2VCU_BRAKE_PRESS_REQUEST_pct = latest_drive_command_->brake_pct;
  } else {
    // Zero all drive outputs
    if (state_machine_.should_forward_commands() && !is_drive_command_fresh()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Drive command on /vehicle/drive_command is stale (>%dms) — zeroing all outputs",
        stale_command_timeout_ms_);
    }
    ai2vcu.AI2VCU_STEER_ANGLE_REQUEST_deg = 0.0f;
    ai2vcu.AI2VCU_AXLE_SPEED_REQUEST_rpm  = 0.0f;
    ai2vcu.AI2VCU_AXLE_TORQUE_REQUEST_Nm  = 0.0f;
    ai2vcu.AI2VCU_BRAKE_PRESS_REQUEST_pct = 0.0f;
  }

  fs_ai_api_ai2vcu_set_data(&ai2vcu);
}

// ── Subscriber callbacks ──────────────────────────────────────────────────────

void VehicleInterfaceNode::drive_command_callback(
  fsai_interfaces::msg::DriveCommand::SharedPtr msg)
{
  latest_drive_command_    = msg;
  last_drive_command_time_ = this->now();
}

void VehicleInterfaceNode::mission_complete_callback(
  std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    RCLCPP_INFO(this->get_logger(),
      "Mission complete received on /vehicle/mission_complete");
    mission_complete_received_ = true;
  }
}

void VehicleInterfaceNode::estop_callback(std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && !estop_requested_) {
    RCLCPP_WARN(this->get_logger(),
      "EBS triggered by software request on /vehicle/estop — "
      "ESTOP_YES will be sent every tick until VCU power cycle");
    estop_requested_ = true;
  }
}

// ── Publish helpers ───────────────────────────────────────────────────────────

void VehicleInterfaceNode::publish_vcu_status(const fs_ai_api_vcu2ai & vcu2ai)
{
  auto msg = fsai_interfaces::msg::VcuStatus();
  msg.header.stamp    = this->now();
  msg.header.frame_id = "";

  msg.as_state  = static_cast<uint8_t>(vcu2ai.VCU2AI_AS_STATE);
  msg.ami_state = static_cast<uint8_t>(vcu2ai.VCU2AI_AMI_STATE);
  msg.res_go_signal        = (vcu2ai.VCU2AI_RES_GO_SIGNAL == RES_GO_SIGNAL_GO);
  msg.handshake_receive_bit = (vcu2ai.VCU2AI_HANDSHAKE_RECEIVE_BIT == HANDSHAKE_RECEIVE_BIT_ON);

  // Fault bits: not exposed by fs_ai_api_vcu2ai — reserved for future API extension.
  // See ARCHITECTURE.md Section 13 for details.
  msg.fault_status              = false;
  msg.warning_status            = false;
  msg.mission_status_fault      = false;
  msg.ebs_fault                 = false;
  msg.ai_comms_lost             = false;
  msg.bms_fault                 = false;
  msg.autonomous_braking_fault  = false;
  msg.brake_plausibility_fault  = false;
  msg.hvil_open_fault           = false;
  msg.hvil_short_fault          = false;
  msg.charge_procedure_fault    = false;
  msg.offboard_charger_fault    = false;
  msg.shutdown_request          = false;
  msg.shutdown_cause            = 0;

  pub_vcu_status_->publish(msg);
}

void VehicleInterfaceNode::publish_wheel_speeds(const fs_ai_api_vcu2ai & vcu2ai)
{
  auto msg = fsai_interfaces::msg::WheelSpeeds();
  msg.header.stamp = this->now();

  msg.fl_rpm = vcu2ai.VCU2AI_FL_WHEEL_SPEED_rpm;
  msg.fr_rpm = vcu2ai.VCU2AI_FR_WHEEL_SPEED_rpm;
  msg.rl_rpm = vcu2ai.VCU2AI_RL_WHEEL_SPEED_rpm;
  msg.rr_rpm = vcu2ai.VCU2AI_RR_WHEEL_SPEED_rpm;

  msg.fl_pulse_count = vcu2ai.VCU2AI_FL_PULSE_COUNT;
  msg.fr_pulse_count = vcu2ai.VCU2AI_FR_PULSE_COUNT;
  msg.rl_pulse_count = vcu2ai.VCU2AI_RL_PULSE_COUNT;
  msg.rr_pulse_count = vcu2ai.VCU2AI_RR_PULSE_COUNT;

  pub_wheel_speeds_->publish(msg);
}

void VehicleInterfaceNode::publish_steering(const fs_ai_api_vcu2ai & vcu2ai)
{
  auto msg  = std_msgs::msg::Float32();
  msg.data  = vcu2ai.VCU2AI_STEER_ANGLE_deg;
  pub_steering_->publish(msg);
}

void VehicleInterfaceNode::publish_brake(const fs_ai_api_vcu2ai & vcu2ai)
{
  auto msg = std_msgs::msg::Float32MultiArray();
  // Index 0 = front, index 1 = rear
  msg.data = {vcu2ai.VCU2AI_BRAKE_PRESS_F_pct, vcu2ai.VCU2AI_BRAKE_PRESS_R_pct};
  pub_brake_->publish(msg);
}

void VehicleInterfaceNode::publish_imu(const fs_ai_api_imu & imu)
{
  auto msg = sensor_msgs::msg::Imu();
  msg.header.stamp    = this->now();
  msg.header.frame_id = imu_frame_id_;

  // Orientation: unknown — set covariance[0] = -1 per ROS convention
  msg.orientation_covariance[0] = -1.0;

  // Linear acceleration: mG → m/s²  (1 G = 9.80665 m/s², 1 mG = 0.00980665 m/s²)
  constexpr double mG_to_ms2 = 9.80665 / 1000.0;
  msg.linear_acceleration.x = static_cast<double>(imu.IMU_Acceleration_X_mG) * mG_to_ms2;
  msg.linear_acceleration.y = static_cast<double>(imu.IMU_Acceleration_Y_mG) * mG_to_ms2;
  msg.linear_acceleration.z = static_cast<double>(imu.IMU_Acceleration_Z_mG) * mG_to_ms2;
  msg.linear_acceleration_covariance[0] = -1.0;

  // Angular velocity: deg/s → rad/s
  constexpr double degps_to_radps = M_PI / 180.0;
  msg.angular_velocity.x = static_cast<double>(imu.IMU_Rotation_X_degps) * degps_to_radps;
  msg.angular_velocity.y = static_cast<double>(imu.IMU_Rotation_Y_degps) * degps_to_radps;
  msg.angular_velocity.z = static_cast<double>(imu.IMU_Rotation_Z_degps) * degps_to_radps;
  msg.angular_velocity_covariance[0] = -1.0;

  pub_imu_->publish(msg);
}

void VehicleInterfaceNode::publish_gps(const fs_ai_api_gps & gps)
{
  auto msg = sensor_msgs::msg::NavSatFix();
  msg.header.stamp    = this->now();
  msg.header.frame_id = gps_frame_id_;

  // Convert degrees + fractional minutes → decimal degrees
  // GPS_Latitude_Degree  = integer degrees
  // GPS_Latitude_Minutes = fractional minutes (not seconds)
  double lat = static_cast<double>(gps.GPS_Latitude_Degree) +
               static_cast<double>(gps.GPS_Latitude_Minutes) / 60.0;
  // 'S' = ASCII 83
  if (gps.GPS_Latitude_IndicatorNS == 83) {
    lat = -lat;
  }

  double lon = static_cast<double>(gps.GPS_Longitude_Degree) +
               static_cast<double>(gps.GPS_Longitude_Minutes) / 60.0;
  // 'W' = ASCII 87
  if (gps.GPS_Longitude_IndicatorEW == 87) {
    lon = -lon;
  }

  msg.latitude  = lat;
  msg.longitude = lon;
  msg.altitude  = static_cast<double>(gps.GPS_Altitude);

  // GPS_NavigationMethod: 0=INIT, 1=NONE, 2=2D fix, 3=3D fix
  msg.status.status =
    (gps.GPS_NavigationMethod >= 2)
    ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
    : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  msg.position_covariance_type =
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

  pub_gps_->publish(msg);
}

void VehicleInterfaceNode::publish_interface_state()
{
  auto msg      = fsai_interfaces::msg::InterfaceState();
  msg.state     = static_cast<uint8_t>(state_machine_.get_state());
  msg.state_name = vehicle_state_name(state_machine_.get_state());
  pub_interface_state_->publish(msg);
}

// ── Helpers ───────────────────────────────────────────────────────────────────

bool VehicleInterfaceNode::is_drive_command_fresh() const
{
  const double age_ms =
    (this->now() - last_drive_command_time_).nanoseconds() / 1.0e6;
  return age_ms < static_cast<double>(stale_command_timeout_ms_);
}

void VehicleInterfaceNode::send_zero_frames()
{
  fs_ai_api_ai2vcu zero = {};
  zero.AI2VCU_ESTOP_REQUEST       = ESTOP_NO;
  zero.AI2VCU_MISSION_STATUS      = MISSION_NOT_SELECTED;
  zero.AI2VCU_DIRECTION_REQUEST   = DIRECTION_NEUTRAL;
  zero.AI2VCU_HANDSHAKE_SEND_BIT  = HANDSHAKE_SEND_BIT_OFF;
  zero.AI2VCU_STEER_ANGLE_REQUEST_deg = 0.0f;
  zero.AI2VCU_AXLE_SPEED_REQUEST_rpm  = 0.0f;
  zero.AI2VCU_AXLE_TORQUE_REQUEST_Nm  = 0.0f;
  zero.AI2VCU_BRAKE_PRESS_REQUEST_pct = 0.0f;

  for (int i = 0; i < 20; ++i) {
    fs_ai_api_ai2vcu_set_data(&zero);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

}  // namespace fsai_vehicle_interface
