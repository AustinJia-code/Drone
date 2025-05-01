cmake_minimum_required (VERSION 3.8)
project (control)

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
find_package(sensor_msgs REQUIRED)

#******************************** INCLUDE PATHS *******************************#
include_directories (include)

#******************************** LIBRARY TARGET ******************************#
add_library (px4_controller src/px4_controller.cpp)

ament_target_dependencies (px4_controller rclcpp px4_msgs)

target_include_directories (px4_controller PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>)

#********************************** EXECUTABLES *******************************#
# Drive node
add_executable (drive_node 
                src/drive_node.cpp)
ament_target_dependencies (drive_node rclcpp px4_msgs sensor_msgs)
install (TARGETS drive_node DESTINATION lib/${PROJECT_NAME})

#*********************************** INSTALL **********************************#
install (TARGETS
         px4_controller
         drive_node
         DESTINATION lib/${PROJECT_NAME})

target_link_libraries(drive_node px4_controller)

# Install header files
install (DIRECTORY include/
         DESTINATION include/)

install (DIRECTORY launch
         DESTINATION share/${PROJECT_NAME}/)

# Export information to downstream packages
ament_export_dependencies (ament_cmake rclcpp rosidl_default_runtime 
                           eigen3_cmake_module Eigen3 px4_msgs geometry_msgs 
                           sensor_msgs)

#************************************ DONE ************************************#
ament_package ()