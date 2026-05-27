#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <fsai_interfaces/msg/drive_command.hpp>
#include <fsai_interfaces/msg/vcu_status.hpp>
#include <fsai_interfaces/msg/wheel_speeds.hpp>
#include <fsai_interfaces/msg/interface_state.hpp>

#include "fsai_vehicle_interface/state_machine.hpp"

namespace fsai_vehicle_interface
{

/**
 * @brief Main ROS 2 node for the FS-AI ADS-DV vehicle interface.
 *
 * Runs a 100 Hz timer loop that:
 *   1. Reads all data from the FS-AI API (VCU, IMU, GPS)
 *   2. Publishes all VCU data as ROS 2 topics
 *   3. Updates the mission state machine
 *   4. Publishes the interface state
 *   5. Forwards drive commands to the VCU via CAN (when in DRIVING state)
 *
 * Subscribes to:
 *   /vehicle/drive_command   [fsai_interfaces/DriveCommand]
 *   /vehicle/mission_complete [std_msgs/Bool]
 *
 * Publishes:
 *   /vcu/status              [fsai_interfaces/VcuStatus]
 *   /vcu/wheel_speeds        [fsai_interfaces/WheelSpeeds]
 *   /vcu/steering            [std_msgs/Float32]
 *   /vcu/brake               [std_msgs/Float32MultiArray]
 *   /vcu/imu                 [sensor_msgs/Imu]
 *   /vcu/gps                 [sensor_msgs/NavSatFix]
 *   /vehicle/interface_state [fsai_interfaces/InterfaceState]
 */
class VehicleInterfaceNode : public rclcpp::Node
{
public:
  explicit VehicleInterfaceNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~VehicleInterfaceNode();

private:
  // ── State ────────────────────────────────────────────────────────────────
  StateMachine state_machine_;
  bool api_initialized_{false};
  bool has_vcu_status_{false};
  bool mission_complete_received_{false};
  rclcpp::Time last_drive_command_time_;
  fsai_interfaces::msg::DriveCommand::SharedPtr latest_drive_command_;

  // ── Parameters ───────────────────────────────────────────────────────────
  std::string can_interface_;
  int stale_command_timeout_ms_;
  std::string imu_frame_id_;
  std::string gps_frame_id_;

  // ── Publishers ───────────────────────────────────────────────────────────
  rclcpp::Publisher<fsai_interfaces::msg::VcuStatus>::SharedPtr       pub_vcu_status_;
  rclcpp::Publisher<fsai_interfaces::msg::WheelSpeeds>::SharedPtr     pub_wheel_speeds_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr                 pub_steering_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr       pub_brake_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr                  pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr            pub_gps_;
  rclcpp::Publisher<fsai_interfaces::msg::InterfaceState>::SharedPtr   pub_interface_state_;

  // ── Subscribers ──────────────────────────────────────────────────────────
  rclcpp::Subscription<fsai_interfaces::msg::DriveCommand>::SharedPtr  sub_drive_command_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                  sub_mission_complete_;

  // ── Timer ────────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr timer_;

  // ── Callbacks ────────────────────────────────────────────────────────────
  void timer_callback();
  void drive_command_callback(fsai_interfaces::msg::DriveCommand::SharedPtr msg);
  void mission_complete_callback(std_msgs::msg::Bool::SharedPtr msg);

  // ── Publish helpers ──────────────────────────────────────────────────────
  void publish_vcu_status(const fs_ai_api_vcu2ai & vcu2ai);
  void publish_wheel_speeds(const fs_ai_api_vcu2ai & vcu2ai);
  void publish_steering(const fs_ai_api_vcu2ai & vcu2ai);
  void publish_brake(const fs_ai_api_vcu2ai & vcu2ai);
  void publish_imu(const fs_ai_api_imu & imu);
  void publish_gps(const fs_ai_api_gps & gps);
  void publish_interface_state();

  // ── Helpers ──────────────────────────────────────────────────────────────
  bool is_drive_command_fresh() const;
  void send_zero_frames();
};

}  // namespace fsai_vehicle_interface
