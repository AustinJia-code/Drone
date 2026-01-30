/**
 * @file auto_patrol.hpp
 * @brief Patrols between waypoints, breaks and tracks target if identified
 */

#pragma once

#include <memory>
#include <algorithm>
#include <cmath>
#include "px4_controller.hpp"
#include "abstract_auto.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

class AutoPatrol : public AutoController
{
public:
  explicit AutoPatrol (std::shared_ptr<PX4Controller> controller,
                     std::shared_ptr<rclcpp::Node> node)
    : AutoController (controller, node)
  {
    target_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped> (
      "/vision/target", 10,
      [this] (const geometry_msgs::msg::PointStamped::SharedPtr msg)
      {
        target_x_ = msg->point.x;
        target_y_ = msg->point.y;
        target_size_ = msg->point.z;
        last_target_time_ = node_->now ();
      });
  }

  ~AutoPatrol () override = default;

  void init () override
  {
    // Set init time
    init_time_ = node_->now ();
    last_target_time_ = node_->now ();
    state_ = State::PATROLLING;
    target_waypoint_ = 0;
    target_detected_ = false;
  }

  bool loop () override
  {
    // Check target staleness
    double target_age = (node_->now () - last_target_time_).seconds ();
    target_detected_ = (target_age < target_stale_timeout_);

    switch (state_)
    {
      case State::PATROLLING:
        // Transition to ERRAND on target detection
        if (target_detected_)
        {
          state_ = State::ERRAND;
          errand_start_time_ = node_->now ();
          break;
        }

        // If at waypoint, go to next waypoint (1m, 1m/s, 0.2rad tolerance)
        if (controller_->pose->get ().approx (waypoints_[target_waypoint_], 1, 1, 0.2))
          target_waypoint_ = (target_waypoint_ + 1) % waypoint_count_;

        // Set waypoint target
        controller_->publish_position_setpoint (waypoints_[target_waypoint_]);

        break;

      case State::ERRAND:
      {
        if (target_detected_)
        {
          // Visual servoing via velocity setpoints
          float vx = Kp_fwd_ * (desired_size_ - target_size_);   // approach/retreat
          float vy = Kp_lat_ * target_x_;                        // strafe to center horizontally
          float vz = -Kp_vert_ * target_y_;                      // altitude to center vertically
          float yaw_rate = Kp_yaw_ * target_x_;                  // yaw toward target

          // Clamp velocities
          vx = std::clamp (vx, -max_speed_, max_speed_);
          vy = std::clamp (vy, -max_speed_, max_speed_);
          vz = std::clamp (vz, -max_speed_, max_speed_);
          yaw_rate = std::clamp (yaw_rate, -max_yaw_rate_, max_yaw_rate_);

          controller_->publish_offboard_control_mode (ControlMode::VEL);
          controller_->publish_velocity_setpoint (vx, vy, vz, yaw_rate);
        }
        else
        {
          // Target lost mid-errand: hold position
          Pose current = controller_->pose->get ();
          controller_->publish_offboard_control_mode (ControlMode::POS);
          controller_->publish_position_setpoint (current.x, current.y, current.z, current.yaw);
        }

        // Return to patrolling on timeout
        double elapsed = (node_->now () - errand_start_time_).seconds ();
        if (elapsed > errand_timeout_)
          state_ = State::PATROLLING;

        break;
      }

      case State::DISABLED:
      default:
        break;
    }

    return true;
  }

  void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) override
  {
    // No user input during this auto
    (void) msg;
  }

private:
  static constexpr std::size_t waypoint_count_ = 4;     // Number of waypoints
  static constexpr double errand_timeout_ = 15.0;       // Seconds before returning to patrol
  static constexpr double target_stale_timeout_ = 0.5;  // Seconds before target considered lost

  // Visual servoing gains
  static constexpr float Kp_fwd_  = 1.0f;               // Forward/backward gain
  static constexpr float Kp_lat_  = 1.0f;               // Lateral gain
  static constexpr float Kp_vert_ = 1.0f;               // Vertical gain
  static constexpr float Kp_yaw_  = 0.5f;               // Yaw gain

  static constexpr float desired_size_ = 0.15f;          // Target normalized radius to maintain
  static constexpr float max_speed_ = 1.0f;              // Max velocity (m/s)
  static constexpr float max_yaw_rate_ = 0.5f;           // Max yaw rate (rad/s)

  /**
   * Waypoints to patrol
   */
  std::array<Pose, waypoint_count_> waypoints_ =
  {
    Pose (0, 0, 10, 0),
    Pose (10, 0, 10, 0),
    Pose (10, 10, 10, 0),
    Pose (0, 10, 0, 0)
  };

  /**
   * States of this auto
   */
  enum class State
  {
    DISABLED = 0,
    PATROLLING = 1,
    ERRAND = 2
  };

  rclcpp::Time init_time_;                                // init time of the auto
  rclcpp::Time errand_start_time_;                        // when ERRAND state began
  rclcpp::Time last_target_time_;                         // last time target was seen
  State state_;                                           // Current state of patrol
  std::size_t target_waypoint_;                           // Current target waypoint

  // Vision target state
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  float target_x_ = 0.0f;                                // Normalized horizontal offset [-1,+1]
  float target_y_ = 0.0f;                                // Normalized vertical offset [-1,+1]
  float target_size_ = 0.0f;                              // Normalized radius (size proxy)
  bool target_detected_ = false;                          // Vision target flag
};
