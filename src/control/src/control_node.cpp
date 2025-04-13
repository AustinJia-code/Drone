/**
 * @file control_node.cpp
 * @brief ROS node for drone control
 */

#include "rclcpp/rclcpp.hpp"
#include "px4_controller.hpp"

using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
private:
  std::shared_ptr<PX4Controller> controller_;
  rclcpp::TimerBase::SharedPtr timer_;
  int counter_ = 0;

public:
  ControlNode() : Node ("control_node")
  {
    controller_ = std::make_shared<PX4Controller> (this);

    // Call every 100ms
    timer_ = this->create_wall_timer (100ms, [this] ()
    {
      // Set offboard mode once 10 setpoints are created
      if (counter_ == 10) {
        controller_->set_offboard_mode();
        controller_->arm();
      }

      // Set velocity control
      controller_->publish_offboard_control_mode (PX4Controller::ControlMode::VEL);
      controller_->publish_velocity_setpoint (1.0, 0.0, 0.0, -0.1);

      if (counter_ < 11)
        counter_++;
    });
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
