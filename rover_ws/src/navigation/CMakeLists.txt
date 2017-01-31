cmake_minimum_required(VERSION 2.8.3)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  rover_msgs
  roscpp
  rospy
)

catkin_package(
    CATKIN_DEPENDS rospy rover_msgs roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
