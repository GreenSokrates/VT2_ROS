cmake_minimum_required(VERSION 2.8.3)
project(cell_core)
add_definitions(-std=c++11)

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
  http_server

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

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
#add_message_files(FILES
#  http_msg.msg
#)

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#  examplefile.srv
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

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
  INCLUDE_DIRS include
# LIBRARIES  
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    roscpp
)

## Enable compiler warnings 
set( CMAKE_CXX_FLAGS "-Wall -Wextra" )
set( CMAKE_CXX_FLAGS_DEBUG "-g -O0" )
set( CMAKE_CXX_FLAGS_RELEASE "-O3" )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  SYSTEM 
    ${Boost_INCLUDE_DIR} 
    ${EIGEN_INCLUDE_DIRS}
  include
    ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

## Declare a C++ library
add_library(collisionObject
  src/collisionObject.cpp
)

## Declare a C++ executable
add_executable(pen_montage
  src/pen_montage.cpp
)

add_executable(cell_pub
  src/cell_pub.cpp
)

add_executable(cell_sub
  src/cell_sup.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(collisionObject ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cell_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cell_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pen_montage ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(collisionObject
  ${catkin_LIBRARIES}
)

target_link_libraries(cell_pub
  ${catkin_LIBRARIES}
)

target_link_libraries(cell_sub
  ${catkin_LIBRARIES}
)

target_link_libraries(pen_montage
  ${catkin_LIBRARIES}
  collisionObject

)