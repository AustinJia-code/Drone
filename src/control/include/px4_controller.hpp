#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

class PX4Controller
{
public:
  enum class ControlMode 
  {
    POS = 0,
    VEL = 1
  };

  PX4Controller (rclcpp::Node *node);

  void arm ();
  void disarm ();
  void set_offboard_mode ();

  void publish_offboard_control_mode (ControlMode mode);
  void publish_velocity_setpoint (float vx, float vy, float vz, float yaw_rate);
  void publish_position_setpoint (float x, float y, float z, float yaw);

private:
  rclcpp::Node *node_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_setpoint_pub_;

  void publish_vehicle_command (uint16_t command, float param1 = 0.0, float param2 = 0.0);
  void get_timestape ();
};
