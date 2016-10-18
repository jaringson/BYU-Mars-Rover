cmake_minimum_required(VERSION 2.8.3)
project(arm_kinematics)

find_package(catkin REQUIRED COMPONENTS 
    rospy
    roscpp
	trac_ik_lib 
	eigen_conversions
	std_msgs
	)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    trac_ik_lib
  DEPENDS
    orocos_kdl
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${eigen_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_executable(arm_ik src/ik_node.cpp)
target_link_libraries(arm_ik
    ${catkin_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
)

install(TARGETS arm_ik
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
