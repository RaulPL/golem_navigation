cmake_minimum_required(VERSION 2.8.3)
project(golem_navigation)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  actionlib
  filters
  json_msgs
  laser_geometry
  move_base
  move_base_msgs
  roscpp
  roslib
  rospy
  rqt_gui
  rqt_gui_py
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

find_library(JSONCPP_LIBRARY jsoncpp /usr/lib/x86_64-linux-gnu)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES golem_navigation
 CATKIN_DEPENDS actionlib filters json_msgs laser_geometry move_base move_base_msgs roscpp roslib rospy rqt_gui rqt_gui_py sensor_msgs std_msgs tf visualization_msgs
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pose_sender
  src/pose_sender.cpp
)

add_library(battery_inspector
  src/battery_inspector.cpp
)

add_library(door_checker
  src/door_checker.cpp
)

add_executable(pose_sender_node src/pose_sender_node.cpp)

add_executable(battery_inspector_node src/battery_inspector_node.cpp)

add_executable(door_checker_node src/door_checker_node.cpp)

target_link_libraries(pose_sender
  ${catkin_LIBRARIES}
  ${JSONCPP_LIBRARY}
)

target_link_libraries(battery_inspector
  ${catkin_LIBRARIES}
  ${JSONCPP_LIBRARY}
)

target_link_libraries(door_checker
  ${catkin_LIBRARIES}
  ${JSONCPP_LIBRARY}
)

target_link_libraries(pose_sender_node
  pose_sender
  ${catkin_LIBRARIES}
  ${JSONCPP_LIBRARY}
)

target_link_libraries(battery_inspector_node
   battery_inspector
   ${catkin_LIBRARIES}
)

target_link_libraries(door_checker_node
   door_checker
   yaml-cpp
   ${catkin_LIBRARIES}
)
