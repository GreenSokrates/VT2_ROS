cmake_minimum_required(VERSION 2.8.3)
project(http_server)
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  cell_core
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
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
#  LIBRARIES http_server
 CATKIN_DEPENDS
   roscpp
   message_runtime
#  DEPENDS system_lib
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
add_library(HTTPServer
  src/HTTPServer.cpp
  src/Thread.cpp
)
add_library(HTTPScript_montage
  src/HTTPScript_montage.cpp 
  )
add_library(HTTPScript_status
  src/HTTPScript_status.cpp
)
add_library(HTTPScript
  src/HTTPScript.cpp 
  )
## Declare a C++ executable
add_executable(HTTPNode
  src/HTTPNode.cpp
  src/HTTPScript_montage.cpp
  src/HTTPScript_status.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(HTTPNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(HTTPServer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(HTTPScript_montage ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(HTTPScript_status ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(HTTPScript ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(HTTPNode
  ${catkin_LIBRARIES}
  HTTPScript_montage
  HTTPScript_status
  HTTPScript
  HTTPServer
)
