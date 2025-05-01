/**
 * @file auto_rose.hpp
 * @brief Rises to 1m for 10 seconds
 */

#pragma once

#include <memory>
#include "px4_controller.hpp"
#include "auto_controller.hpp"
#include "sensor_msgs/msg/joy.hpp"

class AutoRise : public AutoController
{
public:
  explicit AutoRise (std::shared_ptr<PX4Controller> controller,
                     std::shared_ptr<rclcpp::Node> node)
    : AutoController (controller, node), run_time_ (10) {}

  ~AutoRise () override = default;

  void init () override
  {
    // Set init time
    init_time_ = node_->now (); 
  }
  
  bool loop () override
  {
    // Simple rising
    controller_->publish_position_setpoint (0.0, 0.0, 1.0, 0.0);

    // Check if runtime is up
    is_over_ = is_over_ || ((node_->now () - init_time_).seconds () > run_time_);
    return !is_over_;
  }
  
  void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) override 
  { 
    // No user input during this auto
    (void) msg;
  }

private:
  rclcpp::Time init_time_;                      // init time of the auto
  float run_time_;                              // current run time of the auto
};
