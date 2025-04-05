/**
 * @file control_node.cpp
 * @brief ROS node for drone control
 */

// Conditionally set mode based on inputs
// Follow said mode's control scheme
// If gamepad is touched at all, switch to manual

// https://docs.px4.io/main/en/ros2/offboard_control.html