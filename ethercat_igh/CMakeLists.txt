cmake_minimum_required(VERSION 2.8.3)
project(ethercat_igh)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

catkin_package(
INCLUDE_DIRS include
LIBRARIES ethercat_igh 
#  CATKIN_DEPENDS hardware_interface roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(ethercat_igh
  src/ethercat_igh.c
)

## Specify libraries to link a library or executable target against
target_link_libraries(ethercat_igh ethercat ${catkin_LIBRARIES})

