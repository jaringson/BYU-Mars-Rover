cmake_minimum_required(VERSION 2.8.3)
project(vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
)
find_package(OpenCV REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vision
#  CATKIN_DEPENDS roscp std_msgs
#  DEPENDS system_lib
)

include_directories(include)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES velocity_planner
#  CATKIN_DEPENDS relative_nav_msgs roscpp
#  DEPENDS system_lib
)

add_executable(vision_node
                      src/vision_node.cpp
                      src/vision.cpp
                      include/vision.h
                      )

target_link_libraries(vision_node 
		${catkin_LIBRARIES} 
		${OpenCV_LIBS}
)
