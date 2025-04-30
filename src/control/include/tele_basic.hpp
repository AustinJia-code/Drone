/**
 * @file tele_basic.hpp
 * @brief Structure for basic teleop
 */

#pragma once

#include <memory>
#include "px4_controller.hpp"
#include "tele_controller.hpp"
#include "sensor_msgs/msg/joy.hpp"

class TeleBasic : public TeleController
{
public:
  explicit TeleBasic (std::shared_ptr<PX4Controller> controller,
                      std::shared_ptr<rclcpp::Node> node);
  ~TeleBasic () override = default;

  bool loop () override;

protected:
  void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) override;

private:
  /**
   * Joystick values
   */
  struct Joysticks
  {
    float left_x;
    float left_y;
    float right_x;
    float right_y;
  };
  
  Joysticks joysticks_;
};
