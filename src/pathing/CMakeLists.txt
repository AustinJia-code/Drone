cmake_minimum_required (VERSION 3.8)
project (pathing)

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
find_package (sensor_msgs REQUIRED)

#******************************** LIBRARY TARGET ******************************#
add_library (pathing INTERFACE)

# If later add .cpp files:
# add_library(pathing src/pose.cpp)

target_include_directories (pathing INTERFACE
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>)

target_link_libraries (pathing INTERFACE rclcpp)

ament_export_include_directories (include)
ament_export_dependencies (
  rclcpp
  Eigen3)

#*********************************** INSTALL **********************************#
install (DIRECTORY include/
         DESTINATION include/)

#************************************ DONE ************************************#
ament_package ()