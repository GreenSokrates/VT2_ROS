cmake_minimum_required(VERSION 2.8.3)
project(cell_core)
# Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  cmake_modules
)
find_package(Boost REQUIRED
  system
  filesystem
  date_time
  thread
)
find_package(
  Eigen3 REQUIRED
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    roscpp
)


###########
## Build ##
###########

include_directories(
  SYSTEM 
    ${Boost_INCLUDE_DIR} 
    ${EIGEN_INCLUDE_DIRS}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/planning.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(
  TARGETS 
    ${PROJECT_NAME}_node
  DESTINATION 
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)