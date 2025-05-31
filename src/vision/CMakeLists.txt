cmake_minimum_required (VERSION 3.8)
project (vision)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options (-Wall -Wextra -Wpedantic)
endif ()

#****************************** FIND DEPENDENCIES *****************************#
find_package (ament_cmake REQUIRED)
find_package (builtin_interfaces REQUIRED)
find_package (eigen3_cmake_module REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (rclcpp REQUIRED)

#******************************** SET UP TARGETS ******************************#
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