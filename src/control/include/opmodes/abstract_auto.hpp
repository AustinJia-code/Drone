/**
 * @file abstract_auto.hpp
 * @brief Structure for a mostly-auto controller
 */

#pragma once

#include "px4_controller.hpp"
#include "sensor_msgs/msg/joy.hpp"

/**
 * Auto controller
 */
class AutoController
{
public:
  /**
   * Constructor
   */
  AutoController (std::shared_ptr<PX4Controller> controller,
                  std::shared_ptr<rclcpp::Node> node)
    : controller_ (controller), node_ (node) {}
  
  /**
   * Destructor
   */
  virtual ~AutoController () = default;

  /**
   * Initialize the controller
   */
  virtual void init () = 0;

  /**
   * Control loop for the auto
   * @return false if the loop is to terminate, true otherwise
   */
  virtual bool loop () = 0;

  /**
   * Handles minimal user inputs
   */
  virtual void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) = 0;

  /**
   * Getter for completion state
   * @return true if the control loop is complete, false otherwise
   */
  bool is_over () { return is_over_; }

protected:
  std::shared_ptr<PX4Controller> controller_;             // PX4 controller
  std::shared_ptr<rclcpp::Node> node_;                    // Node for pub/subs
  bool is_over_ = false;                                  // Completion state
};