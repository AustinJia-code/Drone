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
    state = State::PATROLLING;
    target_waypoint_ = 0;
  }
  
  bool loop () override
  {
    switch (state)
    {
      case State::PATROLLING:
        // TODO: add errand break on vision id

        // If at waypoint, go to next waypoint (1m, 1m/s, 0.2rad tolerance)
        if (controller_.pose.approx (waypoints [target_waypoint_], 1, 1, 0.2))
          target_waypoint_ = (target_waypoint_ + 1) % waypoint_count_;
        
        // Set waypoint target
        controller_->publish_position_setpoint (waypoints [target_waypoint_]);
        
        break;
      
      case State::ERRAND:
        // CHASE!
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

  /**
   * Waypoints to patrol
   */
  std::array<Pose, waypoint_count_> waypoints = 
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
   State state;                                          // Current state of patrol
   std::size_t target_waypoint_;                         // Current target waypoint
};
