/**
 * @file red_detector_node.cpp
 * @brief Detects red circles in camera feed and publishes normalized target coordinates
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>

class RedDetectorNode : public rclcpp::Node
{
public:
  RedDetectorNode () : Node ("red_detector_node")
  {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image> (
      "/camera/image_raw", rclcpp::SensorDataQoS (),
      std::bind (&RedDetectorNode::image_callback, this, std::placeholders::_1));

    target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped> (
      "/vision/target", 10);

    RCLCPP_INFO (this->get_logger (), "Red detector node started");
  }

private:
  void image_callback (const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert to BGR via cv_bridge
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare (msg, "bgr8");
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 2000,
                            "cv_bridge exception: %s", e.what ());
      return;
    }

    const cv::Mat &frame = cv_ptr->image;
    if (frame.empty ())
      return;

    // Convert to HSV
    cv::Mat hsv;
    cv::cvtColor (frame, hsv, cv::COLOR_BGR2HSV);

    // Two-range red mask to handle HSV hue wraparound
    cv::Mat mask_low, mask_high, mask;
    cv::inRange (hsv, cv::Scalar (0, 120, 70), cv::Scalar (10, 255, 255), mask_low);
    cv::inRange (hsv, cv::Scalar (170, 120, 70), cv::Scalar (180, 255, 255), mask_high);
    mask = mask_low | mask_high;

    // Morphological cleanup
    cv::Mat kernel = cv::getStructuringElement (cv::MORPH_ELLIPSE, cv::Size (5, 5));
    cv::erode (mask, mask, kernel);
    cv::dilate (mask, mask, kernel);

    // Hough circle detection
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles (mask, circles, cv::HOUGH_GRADIENT, 1.5,
                      mask.rows / 4.0,  // min distance between centers
                      100, 20,          // Canny threshold, accumulator threshold
                      10, 0);           // min/max radius (0 = unlimited max)

    if (circles.empty ())
      return;

    // Select largest circle by radius
    cv::Vec3f best = circles[0];
    for (std::size_t i = 1; i < circles.size (); ++i)
    {
      if (circles[i][2] > best[2])
        best = circles[i];
    }

    // Normalize coordinates to [-1, +1] range
    float cx = best[0];
    float cy = best[1];
    float r  = best[2];

    float norm_x = (cx / frame.cols) * 2.0f - 1.0f;
    float norm_y = (cy / frame.rows) * 2.0f - 1.0f;
    float norm_r = (r * 2.0f) / std::min (frame.cols, frame.rows);

    // Publish target
    geometry_msgs::msg::PointStamped target;
    target.header = msg->header;
    target.point.x = norm_x;
    target.point.y = norm_y;
    target.point.z = norm_r;
    target_pub_->publish (target);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
};

int main (int argc, char *argv[])
{
  rclcpp::init (argc, argv);
  rclcpp::spin (std::make_shared<RedDetectorNode> ());
  rclcpp::shutdown ();
  return 0;
}
