cmake_minimum_required(VERSION 2.8.3)
project(ros_socket_can_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  message_generation
)
find_package(Boost REQUIRED COMPONENTS 
  system
)

add_message_files(
  FILES
  CANFrame.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

add_definitions("-D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\"")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS} include/
)

add_executable(
  ros_socket_can_driver_node
  src/ros_socket_can_driver.cpp
)

target_link_libraries(
  ros_socket_can_driver_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  yaml-cpp
)

add_dependencies( 
  ros_socket_can_driver_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
