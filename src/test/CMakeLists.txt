cmake_minimum_required (VERSION 3.8)
project (test)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options (-Wall -Wextra -Wpedantic)
endif ()

#****************************** FIND DEPENDENCIES *****************************#
find_package (ament_cmake REQUIRED)
find_package (builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package (px4_msgs REQUIRED)

#******************************** SET UP TARGETS ******************************#
# Control node
add_executable (test_subscriber src/test_subscriber.cpp)
ament_target_dependencies (test_subscriber rclcpp std_msgs)
install (TARGETS test_subscriber DESTINATION lib/${PROJECT_NAME})

add_executable (test_publisher src/test_publisher.cpp)
ament_target_dependencies (test_publisher rclcpp std_msgs)
install (TARGETS test_publisher DESTINATION lib/${PROJECT_NAME})

add_executable (rise_5m src/rise_5m.cpp)
ament_target_dependencies (rise_5m rclcpp std_msgs px4_msgs)
install (TARGETS rise_5m DESTINATION lib/${PROJECT_NAME})

#*********************************** INSTALL **********************************#
# Install launch files.
install (DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

#************************************ DONE ************************************#
ament_package ()