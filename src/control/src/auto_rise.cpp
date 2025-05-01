/**
 * @file auto_rise.cpp
 * @brief Rises to 1m, completes in 10 seconds
 */

#include "auto_rise.hpp"

// Init with 10 seconds
AutoRise::AutoRise (std::shared_ptr<PX4Controller> controller,
                    std::shared_ptr<rclcpp::Node> node)
  : AutoController (controller, node), run_time_ (10) {}

// Set init time
void AutoRise::init () { init_time_ = node_->now (); }

bool AutoRise::loop ()
{
  // Simple rising
  controller_->publish_position_setpoint (0.0, 0.0, 1.0, 0.0);

  // Check if runtime is up
  is_over_ = is_over_ || ((node_->now () - init_time_).seconds () > run_time_);
  return !is_over_;
}

// No user input during this auto
void AutoRise::joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) { (void) msg; }
