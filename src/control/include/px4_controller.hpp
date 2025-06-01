/**
 * @file px4_controller.hpp
 * @brief PX4 interface to communivate to flight controller
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include "../../pathing/include/pose.hpp"

/**
 * Control modes for the PX4
 */
enum class ControlMode 
{
  POS = 0,  
  VEL = 1
};


class PX4Controller
{
public:
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
  void publish_offboard_control_mode (const ControlMode& mode);

  /**
   * Publish a velocity setpoint command
   */
  void publish_velocity_setpoint (float vx, float vy, float vz, float yaw_rate);

  /**
   * Publish a velocity setpoint from pose
   */
  void publish_velocity_setpoint (const Pose pose);

  /**
   * Publish a position setpoint command
   */
  void publish_position_setpoint (float x, float y, float z, float yaw);

  /**
   * Publish a position setpoint from pose
   */
  void publish_position_setpoint (const Pose pose);

  // Drone pose
  std::shared_ptr<SharedPose> pose;

private:
  // Node
  rclcpp::Node *node_;
  // Command publisher
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
  // Mode publisher
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  // Setpoint publisher
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_setpoint_pub_;

  // Position listener
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_; 
  // Heading listener
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr yaw_sub_; 

  /**
   * Publish command helper
   */
  void publish_vehicle_command (uint16_t command, float param1 = 0.0, float param2 = 0.0);

  /**
   * Timestamp
   */
  void get_timestamp ();
};
