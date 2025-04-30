/**
 * @file auto_rise.cpp
 * @brief Implementation of rise auto
 */

#include "auto_rise.hpp"

/**
 * AutoRise constructor
 */
AutoRise::AutoRise (std::shared_ptr<PX4Controller> controller,
                    std::shared_ptr<rclcpp::Node> node)
  : AutoController (controller, node), run_time_ (10) {}

/**
 * Init
 */
void AutoRise::init () { init_time_ = node_->now (); }

/**
* Auto loop
*/
bool AutoRise::loop ()
{
  // Simple rising
  controller_->publish_position_setpoint (0.0, 0.0, 1.0, 0.0);

  // Check if runtime is up
  is_over_ = is_over_ || ((node_->now () - init_time_).seconds () > run_time_);
  return is_over_;
}

/**
 * Check if auto is over (done/errored)
 */
bool AutoRise::is_over () { return is_over_; }

/**
 * Manual override handling
 */
void AutoRise::joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Check if any input has been received
  is_over_ = std::any_of (msg->axes.begin (), msg->axes.end (), [] (float a) 
                            { return std::abs (a) > 0.05; }) ||
             std::any_of (msg->buttons.begin (), msg->buttons.end (), [] (int b)
                            { return b != 0; });
}
