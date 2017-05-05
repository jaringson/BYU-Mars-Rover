cmake_minimum_required(VERSION 2.8.3)
project(ground_station)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  message_generation
  message_runtime
  std_msgs
)

catkin_python_setup()

add_message_files(
   FILES
   FloatList.msg
   FW_State.msg
   FW_Waypoint.msg
   GPS.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

#
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/ground_station
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
