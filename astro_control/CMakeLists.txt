cmake_minimum_required(VERSION 2.8.3)
project(astro_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  robot_state_publisher
  roscpp rospy std_msgs genmsg
)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(controller_publisher src/controller_publisher.cpp)
target_link_libraries(controller_publisher ${catkin_LIBRARIES})
