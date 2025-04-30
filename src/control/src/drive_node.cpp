/**
 * @file drive_node.cpp
 * @brief ROS node for flight specific controls
 * 
 * TODO: Add auto
 * TODO: Add switch to tele on joy movement
 */

#include "rclcpp/rclcpp.hpp"
#include "px4_controller.hpp"
#include "tele_basic.hpp"
#include "auto_rise.hpp"
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
    tele_ = std::make_unique<TeleBasic> (controller_, shared_from_this ());
    auto_ = std::make_unique<AutoRise> (controller_, shared_from_this ());
    mode_ = DriveMode::INIT;
    
    // Control loop - called every 100ms
    timer_ = this->create_wall_timer (100ms, [this] ()
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
    });
  }

private:
  // Control vars
  std::shared_ptr<PX4Controller> controller_;
  std::unique_ptr<TeleController> tele_;
  std::unique_ptr<AutoController> auto_;
  rclcpp::TimerBase::SharedPtr timer_;
  int counter_ = 0;

  // Drive mode
  DriveMode mode_;

  /**
   * Build initial setpoints to enter offboard mode
   * 
   * @return whether build is complete
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
