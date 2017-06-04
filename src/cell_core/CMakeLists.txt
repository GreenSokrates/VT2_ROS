cmake_minimum_required(VERSION 2.8.3)
project(cell_core)
# Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

#include_directories(
  #include
 # ${catkin_INCLUDE_DIRS}
#)

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
  INCLUDE_DIRS #include
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

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
##################
## Execuatables ##
##################
add_executable(${PROJECT_NAME}_node
  src/planning.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(simple_planning
  src/simple_planning.cpp
)
add_dependencies(simple_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_planning
  ${catkin_LIBRARIES}
)

add_executable(planning_all
  src/planning_all.cpp
  src/collisionObject.cpp
)
add_dependencies(planning_all ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_all
  ${catkin_LIBRARIES}
)

add_executable(pen_montage
  src/pen_montage.cpp
)
add_dependencies(pen_montage ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pen_montage
  ${catkin_LIBRARIES}
  collisionObject

)

add_library(collisionObject
  src/collisionObject.cpp
)
add_dependencies(collisionObject ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(collisionObject
  ${catkin_LIBRARIES}
)