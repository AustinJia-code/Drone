cmake_minimum_required (VERSION 3.8)
project (localization)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options (-Wall -Wextra -Wpedantic)
endif ()

#****************************** FIND DEPENDENCIES *****************************#
find_package (ament_cmake REQUIRED)
find_package (builtin_interfaces REQUIRED)
find_package (eigen3_cmake_module REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (geometry_msgs REQUIRED)
find_package (px4_msgs REQUIRED)
find_package (rclcpp REQUIRED)
find_package (sensor_msgs REQUIRED)

#******************************** SET UP TARGETS ******************************#
# IMU node
add_executable (imu_filter src/imu_filter.cpp)
ament_target_dependencies (imu_filter rclcpp sensor_msgs px4_msgs)
install (TARGETS imu_filter DESTINATION lib/${PROJECT_NAME})

# GPS node
add_executable (gps_filter src/gps_filter.cpp)
ament_target_dependencies (gps_filter rclcpp sensor_msgs geometry_msgs px4_msgs)
install (TARGETS gps_filter DESTINATION lib/${PROJECT_NAME})

# Camera node
# add_executable (camera_listener src/camera_listener.cpp)
# ament_target_dependencies (camera_listener rclcpp)
# install (TARGETS camera_listener DESTINATION lib/${PROJECT_NAME})

#*********************************** INSTALL **********************************#
# Export information to downstream packages
ament_export_dependencies (ament_cmake rclcpp rosidl_default_runtime 
                           eigen3_cmake_module Eigen3 px4_msgs geometry_msgs 
                           sensor_msgs)

# Install launch files.
install (DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

#************************************ DONE ************************************#
ament_package ()