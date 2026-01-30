/**
 * @file auto_bezier.hpp
 * @brief Follows a smooth cubic Bezier spline path through waypoints
 */

#pragma once

#include <memory>
#include <vector>
#include <cmath>
#include "px4_controller.hpp"
#include "abstract_auto.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "../../pathing/include/bezier_path.hpp"

class AutoBezier : public AutoController
{
public:
  explicit AutoBezier (std::shared_ptr<PX4Controller> controller,
                       std::shared_ptr<rclcpp::Node> node,
                       std::vector<Pose> waypoints,
                       float speed = 2.0f)
    : AutoController (controller, node),
      waypoints_ (std::move (waypoints)),
      speed_ (speed) {}

  ~AutoBezier () override = default;

  void init () override
  {
    path_ = std::make_unique<BezierPath> (waypoints_);
    distance_traveled_ = 0.0f;
    is_over_ = false;
  }

  bool loop () override
  {
    if (!path_)
    {
      is_over_ = true;
      return false;
    }

    // Advance along path (fixed 10 Hz dt = 0.1s)
    distance_traveled_ += speed_ * 0.1f;

    // Check completion
    if (distance_traveled_ >= path_->total_length ())
    {
      is_over_ = true;
      return false;
    }

    // Sample position and tangent
    Vec3 pos = path_->sample (distance_traveled_);
    Vec3 tan = path_->sample_tangent (distance_traveled_);

    // Compute yaw from tangent direction
    float yaw = std::atan2 (tan.y, tan.x);

    controller_->publish_position_setpoint (pos.x, pos.y, pos.z, yaw);

    return true;
  }

  void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) override
  {
    (void) msg;
  }

private:
  std::vector<Pose> waypoints_;
  float speed_;
  float distance_traveled_ = 0.0f;
  std::unique_ptr<BezierPath> path_;
};
