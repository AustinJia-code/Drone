/**
 * @file drive_node.cpp
 * @brief ROS node for flight specific controls
 * 
 * TODO: Add auto
 * TODO: Add switch to tele on joy movement
 */

#include "rclcpp/rclcpp.hpp"
#include "px4_controller.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

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

/**
 * Drive states
 */
enum class DriveMode
{
  INIT = -1,
  AUTO = 0,
  TELE = 1,
  FAIL = 2
};

/**
 * DriveNode
 */
class DriveNode : public rclcpp::Node
{
public:
  DriveNode () : Node ("drive_node")
  {
    // Initialization
    controller_ = std::make_shared<PX4Controller> (this);
    mode_ = DriveMode::INIT;
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy> (
      "/joy", 10,
      std::bind (&DriveNode::joy_callback, this, std::placeholders::_1)
    );

    // Control loop - called every 100ms
    timer_ = this->create_wall_timer (100ms, [this] ()
    {
      switch (mode_)
      {
        // Build initial setpoints
        case (DriveMode::INIT):
          build_setpoints ();
          break;
        // Teleop control loop
        case (DriveMode::TELE):
          teleop ();
          break;
        case (DriveMode::AUTO):
          break;
        case (DriveMode::FAIL):
        default:
          return_home ();
          break;
      }
    });
  }

private:
  // Control vars
  std::shared_ptr<PX4Controller> controller_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int counter_ = 0;

  // Joystick vars
  Joysticks joysticks_;
  rclcpp::Time last_joy_time_;

  // Drive mode
  DriveMode mode_;

  /**
   * Build initial setpoints to enter offboard mode
   */
  void build_setpoints ()
  {
    // Set offboard mode once 10 setpoints are created
    if (counter_ == 10)
    {
      controller_->set_offboard_mode();
      controller_->arm();
      mode_ = DriveMode::AUTO;
    }

    // Set velocity control (1 meter up)
    controller_->publish_position_setpoint (0.0, 0.0, 1.0, 0.0);

    // Increment counter
    if (counter_ < 11)
      counter_++;
  }

  /**
   * Joystick input handling
   */
  void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Store joystick values
    if (msg->axes.size() > 3)
    {
      joysticks_ = {msg->axes[0], msg->axes [1], msg->axes [2], msg->axes [3]};
      last_joy_time_ = this->now();
    }
  }

  /**
   * Teleop control
   */
  void teleop ()
  {
    // Check for disconnection
    if ((this->now () - last_joy_time_).seconds () > 1.0)
    {
      mode_ = DriveMode::FAIL;
      return;
    }

    // Set velocity based on joystick input
    controller_->publish_velocity_setpoint
    (
      joysticks_.left_x, joysticks_.left_y,
      joysticks_.right_x, joysticks_.right_y
    );
  }

  /**
   * Failure handling
   */
  void return_home ()
  {
    controller_->publish_position_setpoint (0, 0, 0, 0);
  }
};

/**
 * Main
 */
int main (int argc, char* argv [])
{
  rclcpp::init (argc, argv);
  rclcpp::spin (std::make_shared<DriveNode> ());

  rclcpp::shutdown ();
  
  return 0;
}
