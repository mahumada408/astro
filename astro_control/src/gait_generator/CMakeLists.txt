cmake_minimum_required(VERSION 2.8.3)

##############
## Building ##
##############

add_library(gait_generator gait_generator.cc)
target_link_libraries(gait_generator ${catkin_LIBRARIES})
