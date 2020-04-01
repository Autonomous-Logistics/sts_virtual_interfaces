cmake_minimum_required(VERSION 2.8.3)
project(sts_virtual_interface_msgs)

## Use C++11
add_definitions(-Wall -lglut -lGL -std=c++17)

# openMP
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

# Set to Release in order to speed up the program significantly
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
# set(CMAKE_BUILD_TYPE Debug)
endif()
# Make sure symbols are there, and not fail at runtime
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    roscpp
    rospy
    std_msgs
    geometry_msgs

    actionlib
    message_generation
    actionlib_msgs #message_generation is referenced implicitly by actionlib_msgs
)

###################################################

#Messages, Services and Actions inside of SUBDIRS WILL NOT BE VISIBLE via rossrv list !!!

add_message_files(
DIRECTORY msg
FILES
  sts_health_monitor_interface/HealthReport.msg
)

add_service_files( 
DIRECTORY srv
FILES 
  sts_behavior_tree_interface/BTService.srv
  sts_health_monitor_interface/HealthCheck.srv
  sts_health_monitor_interface/HealthReset.srv
)

add_action_files( 
DIRECTORY action
FILES 
  sts_behavior_tree_interface/BT.action
)


###################################################

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${EXTERNAL_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}_core actionlib
  CATKIN_DEPENDS std_msgs message_runtime geometry_msgs actionlib_msgs
)


###########
## Build ##
###########
file(GLOB_RECURSE HeaderFiles "include/*.h")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_core
  ${HeaderFiles}
)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

#we need this because we dont have a cpp file ... so cmake dosnt now what to do with this lib :D
#CMake Error: Cannot determine link language for target "sts_planning_interface_msgs".
#...
set_target_properties(${PROJECT_NAME}_core PROPERTIES LINKER_LANGUAGE CXX)




#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation

# we have no doc folder
#install(
#  DIRECTORY doc
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

