cmake_minimum_required(VERSION 2.8.3)
project(pioneer_teleop)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp)
find_package(Boost REQUIRED COMPONENTS thread)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare catkin package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pioneer_teleop
#  CATKIN_DEPENDS geometry_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## cpp executables and libs
add_executable(keyboard src/keyboard.cpp)
target_link_libraries(keyboard ${catkin_LIBRARIES})
