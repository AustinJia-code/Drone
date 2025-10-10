/**
 * @file test_odo.cpp
 * @brief ROS node for PX4 odometry
 */

#include "rclcpp/rclcpp.hpp"
#include <px4_msgs/msg/vehicle_local_position.hpp>

class PositionListener : public rclcpp::Node
{
public:
  explicit PositionListener () : Node ("position_listener")
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS (
        rclcpp::QoSInitialization (qos_profile.history, 5),
        qos_profile);

    subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition> (
        "/fmu/out/vehicle_local_position", qos,
        [this] (const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
        {
          std::cout << "\033[2J\033[1;1H"; // clear terminal for readability
          std::cout << "RECEIVED LOCAL POSITION DATA\n";
          std::cout << "============================\n";
          std::cout << "timestamp: " << msg->timestamp << "\n";
          std::cout << "x: " << msg->x << " m\n";
          std::cout << "y: " << msg->y << " m\n";
          std::cout << "z: " << msg->z << " m\n";
          std::cout << "vx: " << msg->vx << " m/s\n";
          std::cout << "vy: " << msg->vy << " m/s\n";
          std::cout << "vz: " << msg->vz << " m/s\n";
          std::cout << "heading: " << msg->heading << " rad\n";
          std::cout << "xy_valid: " << (msg->xy_valid ? "true" : "false") << "\n";
          std::cout << "z_valid: " << (msg->z_valid ? "true" : "false") << "\n";
        });
  }

private:
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
};

int main (int argc, char *argv[])
{
  std::cout << "PX4 Local Position Listener" << std::endl;
  setvbuf (stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init (argc, argv);
  rclcpp::spin (std::make_shared<PositionListener>());
  rclcpp::shutdown ();
  
  return 0;
}
