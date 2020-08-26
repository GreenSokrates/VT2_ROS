cmake_minimum_required(VERSION 2.8.3)
project(smc_grippers)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    std_msgs
    message_generation
    genmsg
)

# System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS 
    system
    filesystem
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate services in the 'srv' folder
 add_service_files(
   FILES
    gripper_service.srv
    press_service.srv
    Bgripper_service.srv
)

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  LIBRARIES smc_grippers
  CATKIN_DEPENDS
    message_runtime
    roscpp
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
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
add_library(CoE
    src/CoE.cpp
    src/RealtimeThread.cpp
)
add_library(EtherCAT
    src/EtherCAT.cpp
)
add_library(SMCServo 
    src/SMCServoJXCE1.cpp
    src/Thread.cpp
    src/Mutex.cpp
    src/Timer.cpp
)
## Declare a C++ executable

 add_executable(smcNode 
    src/smcService.cpp
)

# Add cmake target dependencies of the executable
# same as for the library above
 add_dependencies(smcNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
 target_link_libraries(smcNode
   ${catkin_LIBRARIES}
   CoE
   EtherCAT
   SMCServo
)
