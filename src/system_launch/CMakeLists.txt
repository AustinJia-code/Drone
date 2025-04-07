cmake_minimum_required (VERSION 3.8)
project (system_launch)

#****************************** FIND DEPENDENCIES *****************************#
find_package (ament_cmake REQUIRED)

#*********************************** INSTALL **********************************#
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

#************************************ DONE ************************************#
ament_package ()