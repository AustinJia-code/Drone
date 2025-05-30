/**
 * @file drive_node.cpp
 * @brief ROS node for flight specific controls - contains one teleop and auto
 */

#include "rclcpp/rclcpp.hpp"
#include "px4_controller.hpp"
#include "opmodes/tele_basic.hpp"
#include "opmodes/auto_rise.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

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
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>
    (
      "/joy", 10,
      std::bind (&DriveNode::joy_callback, this, std::placeholders::_1)
    );
    mode_ = DriveMode::INIT;
    
    // Control loop - called every 100ms
    timer_ = this->create_wall_timer (100ms, [this] () { loop (); });
  }

  /**
   * Init controllers after node has been built
   */
  void initialize ()
  {
    tele_ = std::make_unique<TeleBasic> (controller_, shared_from_this ());
    auto_ = std::make_unique<AutoRise> (controller_, shared_from_this ());
  }

private:
  // Control vars
  std::shared_ptr<PX4Controller> controller_;
  std::unique_ptr<TeleController> tele_;
  std::unique_ptr<AutoController> auto_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  int counter_ = 0;

  // Drive mode
  DriveMode mode_;

  /**
   * Build initial setpoints to enter offboard mode
   * @return true if build is complete, false otherwise
   */
  bool build_setpoints ()
  {
    // Set offboard mode once 10 setpoints are created
    if (counter_ == 10)
    {
      controller_->set_offboard_mode ();
      controller_->arm ();
      
      return true;
    }

    // Set velocity control (1 meter up)
    controller_->publish_position_setpoint (0.0, 0.0, 1.0, 0.0);

    // Increment counter
    if (counter_ < 11)
      counter_++;

    // Build incomplete
    return false;
  }

  /**
   * Failure handling
   */
  void return_home () { controller_->publish_position_setpoint (0, 0, 0, 0); }

  /**
   * State machine
   */
  void loop ()
  {
    switch (mode_)
    {
      // Build initial setpoints
      case (DriveMode::INIT):
        // Go to tele on completion
        if (build_setpoints ())
          mode_ = DriveMode::TELE;
        break;

      // Teleop control loop
      case (DriveMode::TELE):
        if (!tele_->loop ())
          mode_ = DriveMode::FAIL;
        break;

      // Auto control loop
      case (DriveMode::AUTO):
        // Go to tele on completion or failure
        if (!auto_->loop ())
          mode_ = DriveMode::TELE;
        break;

      // Failure case
      case (DriveMode::FAIL):
      default:
        return_home ();
        break;
    }
  }

  /**
   * Gamepad transitions and msg passthrough
   */
  void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // if 'A' pressed in TELE, switch to AUTO
    if (msg->buttons.size () > 0 && msg->buttons [0] == 1 && mode_ == DriveMode::TELE)
    {
      auto_->init ();
      mode_ = DriveMode::AUTO;
    }

    // if any input in AUTO, switch to TELE
    if (mode_ == DriveMode::AUTO)
    {
      // Check if any input has been received
      bool input = std::any_of (msg->axes.begin (), msg->axes.end (), [] (float a) 
                      { return std::abs (a) > 0.05; }) ||
                   std::any_of (msg->buttons.begin (), msg->buttons.end (), [] (int b)
                      { return b != 0; });

      // Go tele if yes
      if (input)
        mode_ = DriveMode::TELE;
    }

    // Pass joy input to active controller
    switch (mode_)
    {
      // Normal teleop actions
      case DriveMode::TELE: 
        tele_->joy_callback (msg);
        break;
      // Some autos have minimal joy input
      case DriveMode::AUTO:
        auto_->joy_callback (msg);
        break;
      // Default nothing
      default: 
        break;
    }
  }
};

/**
 * Main
 */
int main (int argc, char* argv [])
{
  rclcpp::init (argc, argv);
  auto node = std::make_shared<DriveNode> ();
  node->initialize ();

  rclcpp::spin (node);

  rclcpp::shutdown ();
  
  return 0;
}
