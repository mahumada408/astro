cmake_minimum_required(VERSION 2.8.3)

##############
## Building ##
##############

add_library(floating_base floating_base.cc)
target_link_libraries(floating_base ${catkin_LIBRARIES})
