/**
 * @file px4_controller.cpp
 * @brief PX4 controller interface
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <px4_controller.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

PX4Controller::PX4Controller (rclcpp::Node *node) : node_ (node)
{
  // Handles vehicle commands (arm, disarm, etc.)
  vehicle_cmd_pub_ = node_->create_publisher<VehicleCommand>
                              ("/fmu/in/vehicle_command", 10);
  // Handles switching to offboard mode
  offboard_mode_pub_ = node_->create_publisher<OffboardControlMode>
                                ("/fmu/in/offboard_control_mode", 10);
  // Handles setpoints (position, trajectory, etc.)
  traj_setpoint_pub_ = node_->create_publisher<TrajectorySetpoint>
                                ("/fmu/in/trajectory_setpoint", 10);

  // Init pose
  pose = std::make_shared<SharedPose> ();

  // Read position, store it
  position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>
  (
    "/fmu/out/vehicle_local_position", 10,
    [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
      pose->set_position (msg->x, msg->y, msg->z);
      pose->set_velocity (msg->vx, msg->vy, msg->vz);
    }
  );

  // Read yaw, store it
  yaw_sub_ = node_->create_subscription<px4_msgs::msg::VehicleAttitude>
  (
    "/fmu/out/vehicle_attitude", 10,
    [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
      float q0 = msg->q[0];
      float q1 = msg->q[1];
      float q2 = msg->q[2];
      float q3 = msg->q[3];

      pose->set_yaw (std::atan2 (2.0f * (q0 * q3 + q1 * q2),
                                          1.0f - 2.0f * (q2 * q2 + q3 * q3)));
    }
  );
}

void PX4Controller::publish_vehicle_command (uint16_t command, float param1, 
                                             float param2)
{
  VehicleCommand cmd;
  cmd.timestamp = node_->get_clock ()
                       ->now ()
                        .nanoseconds () 
                       / 1000;
  cmd.param1 = param1;
  cmd.param2 = param2;
  cmd.command = command;
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;
  vehicle_cmd_pub_->publish (cmd);
}

void PX4Controller::arm ()
{
  publish_vehicle_command (VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void PX4Controller::disarm ()
{
  publish_vehicle_command (VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

void PX4Controller::set_offboard_mode ()
{
  publish_vehicle_command (VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

void PX4Controller::publish_offboard_control_mode (const ControlMode& mode)
{
  // Create offboard control message
  OffboardControlMode msg {};
  msg.timestamp = node_->get_clock ()
                       ->now()
                        .nanoseconds()
                       / 1000;
  msg.position = mode == ControlMode::POS;
  msg.velocity = mode == ControlMode::VEL;
  offboard_mode_pub_->publish (msg);

  // Explicitly disable
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
}

void PX4Controller::publish_velocity_setpoint (float vx, float vy, float vz, 
                                               float yaw_rate)
{
  publish_offboard_control_mode (ControlMode::VEL);

  // Create trajectory message
  TrajectorySetpoint msg {};
  msg.timestamp = node_->get_clock ()
                       ->now()
                        .nanoseconds()
                       / 1000;

  // In NED, Z is down
  msg.velocity = {vx, vy, -vz};
  msg.yawspeed = yaw_rate;
  traj_setpoint_pub_->publish(msg);
}

void PX4Controller::publish_position_setpoint (float x, float y, float z, 
                                               float yaw)
{
  publish_offboard_control_mode (ControlMode::POS);

  // Create trajectory message
  TrajectorySetpoint msg{};
  msg.timestamp = node_->get_clock()
                       ->now()
                        .nanoseconds()
                       / 1000;

  // In NED, Z is down
  msg.position = {x, y, -z};
  msg.yaw = yaw;
  traj_setpoint_pub_->publish (msg);
}
