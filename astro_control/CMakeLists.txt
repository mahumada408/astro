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
  tf tf_conversions
)

catkin_package(
  INCLUDE_DIRS include
)
include_directories(include ${catkin_INCLUDE_DIRS})

add_subdirectory(src/floating_base)
add_subdirectory(src/qpOASES)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(controller_publisher src/controller_publisher.cpp)
target_link_libraries(controller_publisher 
                      ${catkin_LIBRARIES}
                      qpOASES
                      floating_base)


