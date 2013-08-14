cmake_minimum_required(VERSION 2.8.3)
project(move_base_simple)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS angles geometry_msgs nav_msgs roscpp tf)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare catkin package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES move_base_simple
#  CATKIN_DEPENDS angles geometry_msgs nav_msgs roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## cpp executables and libs
add_executable(move_base_simple src/move_base_simple.cpp)
target_link_libraries(move_base_simple ${catkin_LIBRARIES})