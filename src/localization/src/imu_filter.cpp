/**
 * @file imu_filter.cpp
 * @brief Publishes IMU onto standard ROS topics
 * @cite https://github.com/PX4/px4_ros_com/blob/main/src/examples/listeners/sensor_combined_listener.cpp
 */

#include "rclcpp/rclcpp.hpp"
#include <px4_msgs/msg/sensor_combined.hpp>
#include <sensor_msgs/msg/imu.hpp>

class IMUFilter : public rclcpp::Node
{
public:
  explicit IMUFilter () : Node ("imu_filter")
  {
    // Subscribe to PX4 IMU
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS (rclcpp::QoSInitialization (qos_profile.history, 5), 
                            qos_profile);

    subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>
    (
      "/fmu/out/sensor_combined", qos, 
      std::bind (&IMUFilter::imu_callback, this, std::placeholders::_1)
    );

    // Set up ROS2 IMU Publisher
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu> ("/imu/data", 10);
  }
 
private:
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  /**
   * Handle IMU passthrough
   */
  void imu_callback (const px4_msgs::msg::SensorCombined::UniquePtr msg)
  {
    // Init message
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->get_clock ()->now ();
    imu_msg.header.frame_id = "base_link";

    // Set velocity
    imu_msg.angular_velocity.x = msg->gyro_rad [0];
    imu_msg.angular_velocity.y = msg->gyro_rad [1];
    imu_msg.angular_velocity.z = msg->gyro_rad [2];

    // Set acceleration
    imu_msg.linear_acceleration.x = msg->accelerometer_m_s2 [0];
    imu_msg.linear_acceleration.y = msg->accelerometer_m_s2 [1];
    imu_msg.linear_acceleration.z = msg->accelerometer_m_s2 [2];

    // Orientation covariance:
    imu_msg.orientation_covariance [0] = -1;

    // Publish
    imu_pub_->publish (imu_msg);
  }
};
 
/**
 * Main
 */
int main (int argc, char * argv [])
{
  std::cout << "IMU filter node" << std::endl;
  setvbuf (stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init (argc, argv);
  rclcpp::spin (std::make_shared <IMUFilter> ());

  rclcpp::shutdown ();
  return 0;
}
