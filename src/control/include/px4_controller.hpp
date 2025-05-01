/**
 * @file px4_controller.hpp
 * @brief PX4 interface to communivate to flight controller
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

/**
 * Drive states
 */
enum class DriveMode
{
  INIT = -1,
  AUTO = 0,
  TELE = 1,
  FAIL = 2
};

class PX4Controller
{
public:
  /**
   * Control modes for the PX4
   */
  enum class ControlMode 
  {
    POS = 0,
    VEL = 1
  };

  /**
   * Constructor
   */
  PX4Controller (rclcpp::Node *node);

  /**
   * Arm the flight controller
   */
  void arm ();

  /**
   * Disarm the flight controller
   */
  void disarm ();
  
  /**
   * Set the flight controller to be controlled in offboard mode
   */
  void set_offboard_mode ();

  /**
   * Publish a control mode command
   */
  void publish_offboard_control_mode (ControlMode mode);

  /**
   * Publish a velocity setpoint command
   */
  void publish_velocity_setpoint (float vx, float vy, float vz, float yaw_rate);

  /**
   * Publish a position setpoint command
   */
  void publish_position_setpoint (float x, float y, float z, float yaw);

private:
  // Node
  rclcpp::Node *node_;
  // Command publisher
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
  // Mode publisher
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  // Setpoint publisher
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_setpoint_pub_;

  /**
   * Publish command helper
   */
  void publish_vehicle_command (uint16_t command, float param1 = 0.0, float param2 = 0.0);

  /**
   * Timestamp getter
   */
  void get_timestape ();
};
