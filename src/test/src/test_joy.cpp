#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>

class JoyListener : public rclcpp::Node
{
public:
  JoyListener () : Node ("joy_listener")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy> 
    (
      "joy", 10,
      std::bind (&JoyListener::joy_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO (this->get_logger (), "JoyListener node started.");
  }

private:
  void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::ostringstream axes, buttons;
    for (float a : msg->axes) axes << a << " ";
    for (int b : msg->buttons) buttons << b << " ";
    RCLCPP_INFO (this->get_logger(), "Axes: [%s]  Buttons: [%s]",
                 axes.str().c_str(), buttons.str().c_str());
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main (int argc, char *argv [])
{
  rclcpp::init (argc, argv);
  
  rclcpp::spin (std::make_shared<JoyListener> ());
  
  rclcpp::shutdown ();
  return 0;
}
