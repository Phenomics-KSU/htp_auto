cmake_minimum_required(VERSION 2.8.3)
project(htp_auto)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
  rospy
  actionlib
  message_generation
  std_msgs
  actionlib_msgs
  nav_msgs
  tf
  dynamic_reconfigure
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Command.msg
  Home.msg
  OdometryUTC.msg
  PIDState.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddMissionItem.srv
  ConvertLLA2ENU.srv
  LoadMissionFile.srv
  PID.srv
  SetHome.srv
  SetMissionItem.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  WaypointGuidance.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  nav_msgs
)

generate_dynamic_reconfigure_options(
  cfg/GPSConverterParams.cfg
  cfg/GuidanceParams.cfg
)

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
#  LIBRARIES htp_auto
  CATKIN_DEPENDS message_runtime actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

## Declare a cpp library
# add_library(htp_auto
#   src/${PROJECT_NAME}/htp_auto.cpp
# )

## Declare a cpp executable
add_executable(waypoint_guidance src/waypoint_guidance.cpp src/pid.cpp)
add_executable(mission src/mission.cpp)
add_executable(mission_loader src/mission_loader.cpp)
add_executable(quat_to_euler src/quat_to_euler.cpp)
add_executable(gps_converter src/gps_converter.cpp src/coordinate_conversions.cpp)
add_executable(log_converter src/log_converter.cpp src/coordinate_conversions.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(waypoint_guidance htp_auto_generate_messages_cpp)
add_dependencies(waypoint_guidance {PROJECT_NAME}_gencfg)
add_dependencies(mission htp_auto_generate_messages_cpp)
add_dependencies(mission_loader htp_auto_generate_messages_cpp)
add_dependencies(quat_to_euler htp_auto_generate_messages_cpp)
add_dependencies(gps_converter htp_auto_generate_messages_cpp)
add_dependencies(gps_converter {PROJECT_NAME}_gencfg)
add_dependencies(log_converter htp_auto_generate_messages_cpp)
add_dependencies(log_converter {PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(waypoint_guidance
   ${catkin_LIBRARIES}
)

target_link_libraries(mission
   ${catkin_LIBRARIES}
)

target_link_libraries(mission_loader
   ${catkin_LIBRARIES}
)

target_link_libraries(quat_to_euler
   ${catkin_LIBRARIES}
)

target_link_libraries(gps_converter
   ${catkin_LIBRARIES}
)

target_link_libraries(log_converter
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
   src/gps_server.py
   src/teleop.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
# install(TARGETS htp_auto htp_auto_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_htp_auto.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
