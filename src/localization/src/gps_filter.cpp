/**
 * @file gps_filter.cpp
 * @brief Publishes PX4 GPS info on standard ROS2 channels
 * @cite https://github.com/PX4/px4_ros_com/blob/main/src/examples/listeners/vehicle_gps_position_listener.cpp
 */

#include "rclcpp/rclcpp.hpp"
#include <px4_msgs/msg/sensor_gps.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

class GPSFilter : public rclcpp::Node
{
public:
  explicit GPSFilter () : Node ("gps_filter")
  {
    // Set up PX4 GPS listener
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS (rclcpp::QoSInitialization (qos_profile.history, 5),
                            qos_profile);
		
		subscription_ = this->create_subscription<px4_msgs::msg::SensorGps>
    (
      "/fmu/out/vehicle_gps_position", qos,
      std::bind (&GPSFilter::gps_callback, this, std::placeholders::_1)
    );

    // Set up publishers
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix> ("/fix", 10);
    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped> ("/gps/vel", 10);
  }

private:
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_pub_;

  /**
   * Handle GPS parsing
   */
  void gps_callback (const px4_msgs::msg::SensorGps::UniquePtr msg)
  {
    // Set up fix message
    sensor_msgs::msg::NavSatFix fix_msg;
    fix_msg.header.stamp = this->get_clock ()->now ();
    fix_msg.header.frame_id = "gps";
    fix_msg.latitude = msg->latitude_deg;
    fix_msg.longitude = msg->longitude_deg;
    fix_msg.altitude = msg->altitude_msl_m;
    fix_msg.status.status = (msg->fix_type >= 3) ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                                                 : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    rclcpp::Time gps_time(msg->timestamp, RCL_ROS_TIME);
    fix_msg.header.stamp = rclcpp::Time (msg->timestamp, RCL_ROS_TIME);
    gps_pub_->publish (fix_msg);

    // Set up velocity message
    geometry_msgs::msg::TwistWithCovarianceStamped vel_msg;
    vel_msg.header = fix_msg.header;
    vel_msg.twist.twist.linear.x = msg->vel_n_m_s;
    vel_msg.twist.twist.linear.y = msg->vel_e_m_s;
    vel_msg.twist.twist.linear.z = msg->vel_d_m_s;
    vel_pub_->publish (vel_msg);
  }
};

/**
 * Main
 */
int main(int argc, char * argv [])
{
  std::cout << "GPS filter node" << std::endl;
  setvbuf (stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init (argc, argv);
  rclcpp::spin (std::make_shared <GPSFilter> ());

  rclcpp::shutdown ();
  return 0;
}
