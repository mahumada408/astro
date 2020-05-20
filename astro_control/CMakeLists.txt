cmake_minimum_required(VERSION 2.8.3)
project(astro_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wall -g)

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

find_package(ct_core REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

catkin_package(
  INCLUDE_DIRS 
    include
  DEPENDS 
    ct_core
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Adds the individual libraries.
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

add_executable(example_optimization src/examples/example_optimization.cc)
target_link_libraries(example_optimization
                      ${catkin_LIBRARIES}
                      floating_base
                      ct_core)

# Unit target
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(testing-test launch/gtest.test test/test_hello.cc)
  target_link_libraries(testing-test ${catkin_LIBRARIES} floating_base)

  add_rostest_gtest(test_astro_ct_optcon launch/gtest_optcon.test test/test_astro_ct_optcon.cc)
  target_link_libraries(test_astro_ct_optcon ${catkin_LIBRARIES} floating_base ct_core)

endif()
