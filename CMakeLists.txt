cmake_minimum_required(VERSION 3.0.2)
project(ping_altimeter_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  dynamic_reconfigure
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
