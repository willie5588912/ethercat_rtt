cmake_minimum_required(VERSION 2.8.3)
project(ethercat_main)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  rtt_rosclock
  ethercat_igh
)

catkin_package(
  CATKIN_DEPENDS rtt_ros 
)

add_definitions(-DRTT_COMPONENT)
orocos_component(${PROJECT_NAME} src/ethercat_main.cpp)
target_link_libraries(${PROJECT_NAME} ethercat_igh ${catkin_LIBRARIES})

orocos_generate_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

