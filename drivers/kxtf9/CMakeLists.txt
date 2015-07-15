cmake_minimum_required(VERSION 2.8.3)
project(kxtf9)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)
find_library(wiringPi wiringPi)
#link_directories(/usr/local/lib)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kxtf9
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

# build only when wiringPi lib is present (only on RaspberryPi)
if (wiringPi)
  ## Specify additional locations of header files
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

  ## Declare a cpp executable
  add_executable(kxtf9 src/kxtf9.cpp)

  ## Specify libraries to link a library or executable target against
  target_link_libraries(kxtf9
    wiringPi
    ${catkin_LIBRARIES}
    )
else (wiringPi)
  message(WARNING "wiringPi library not found. kxtf9 not built.")
endif (wiringPi)
