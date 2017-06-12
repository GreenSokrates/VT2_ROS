cmake_minimum_required(VERSION 2.8.3)
project(http_server)
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES http_server
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

## Enable compiler warnings 
set( CMAKE_CXX_FLAGS "-Wall -Wextra" )
set( CMAKE_CXX_FLAGS_DEBUG "-g -O0" )
set( CMAKE_CXX_FLAGS_RELEASE "-O3" )


###########
## Build ##
###########

include_directories(
  SYSTEM 
    ${Boost_INCLUDE_DIR} 
    ${EIGEN_INCLUDE_DIRS}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
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




add_executable(HTTPNode
  src/HTTPNode.cpp
  src/HTTPServer.cpp
  src/Thread.cpp
)
add_dependencies(HTTPNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(HTTPNode
  ${catkin_LIBRARIES}
)

add_library(HTTPServer
  src/HTTPServer.cpp
)
add_dependencies(HTTPServer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(HTTPServer
  ${catkin_LIBRARIES}
)