cmake_minimum_required(VERSION 2.8.3)
project(cell_core)
# Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  moveit_ros_planning_interface
)
#This tells CMake to planning.cpp and name it correspondingly
add_executable(${PROJECT_NAME}_node src/planning.cpp)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cell_core
CATKIN_DEPENDS
  roscpp
  geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_node
${catkin_LIBRARIES}
${Boost_LIBRARIES}
 
)

#############
## Install ##
#############
