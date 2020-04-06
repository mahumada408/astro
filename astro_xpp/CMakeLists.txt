cmake_minimum_required(VERSION 2.8.3)
project(astro_xpp)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  message_generation
  xpp_vis
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_library(${PROJECT_NAME}
  src/astroleg_inverse_kinematics.cc
  src/inverse_kinematics_astro.cc
)

## URDF visualizer
add_executable(astro_xpp_visualizer src/astro_xpp_visualizer.cc)
target_link_libraries(astro_xpp_visualizer
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
# Mark library for installation
install(
  TARGETS ${PROJECT_NAME} astro_xpp_visualizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY launch rviz meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)