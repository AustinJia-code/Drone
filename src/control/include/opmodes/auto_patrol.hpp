/**
 * @file auto_patrol.hpp
 * @brief Patrols between waypoints, breaks and tracks target if identified
 */

#pragma once

#include <memory>
#include "px4_controller.hpp"
#include "abstract_auto.hpp"
#include "sensor_msgs/msg/joy.hpp"

class AutoPatrol : public AutoController
{
public:
  explicit AutoPatrol (std::shared_ptr<PX4Controller> controller,
                     std::shared_ptr<rclcpp::Node> node)
    : AutoController (controller, node) {}

  ~AutoPatrol () override = default;

  void init () override
  {
    // Set init time
    init_time_ = node_->now ();
    state_ = State::PATROLLING;
    target_waypoint_ = 0;
    target_detected_ = false;
  }

  bool loop () override
  {
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
        // Hold current position (placeholder for vision-driven pursuit)
        Pose current = controller_->pose->get ();
        controller_->publish_position_setpoint (current.x, current.y, current.z, current.yaw);

        // Return to patrolling on timeout or target lost
        double elapsed = (node_->now () - errand_start_time_).seconds ();
        if (elapsed > errand_timeout_ || !target_detected_)
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

  void set_target_detected (bool detected) { target_detected_ = detected; }

private:
  static constexpr std::size_t waypoint_count_ = 4;     // Number of waypoints
  static constexpr double errand_timeout_ = 15.0;       // Seconds before returning to patrol

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

   rclcpp::Time init_time_;                              // init time of the auto
   rclcpp::Time errand_start_time_;                      // when ERRAND state began
   State state_;                                         // Current state of patrol
   std::size_t target_waypoint_;                         // Current target waypoint
   bool target_detected_ = false;                        // Vision target flag
};
