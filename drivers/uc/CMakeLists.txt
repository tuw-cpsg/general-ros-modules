cmake_minimum_required(VERSION 2.8.3)
project(uc)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs
  additional_msgs
  )
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
#  LIBRARIES uc
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

# build only when wiringPi lib is present (only on RaspberryPi)
if (wiringPi)
  ## Specify additional locations of header files
  include_directories(
    ${catkin_INCLUDE_DIRS}
    )

  ## Declare a cpp executable
  add_executable(uc_node src/uc_node.cpp)

  ## Specify libraries to link a library or executable target against
  target_link_libraries(uc_node
    wiringPi
    ${catkin_LIBRARIES}
    )
else (wiringPi)
  message(WARNING "wiringPi library not found. uc not built.")
endif (wiringPi)
