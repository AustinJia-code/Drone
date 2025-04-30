/**
 * @file auto_rose.hpp
 * @brief Structure for auto that rises to 1m
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
                    std::shared_ptr<rclcpp::Node> node);
  ~AutoRise () override = default;

  void init () override;
  bool loop () override;
  bool is_over () override;

protected:
  void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) override;

private:
  rclcpp::Time init_time_;
  float run_time_;
};
