cmake_minimum_required(VERSION 2.8.3)
project(dji_sdk)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
  std_srvs
  sensor_msgs
  tf
  actionlib_msgs
  actionlib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

add_message_files(
  FILES
  Acceleration.msg
  AttitudeQuaternion.msg
  Compass.msg
  FlightControlInfo.msg
  Gimbal.msg
  GlobalPosition.msg
  LocalPosition.msg
  PowerStatus.msg
  RCChannels.msg
  Velocity.msg
  Waypoint.msg
  WaypointList.msg

  TimeStamp.msg

  MissionPushInfo.msg
  MissionWaypointAction.msg
  MissionWaypoint.msg
  MissionWaypointTask.msg
  MissionHotpointTask.msg
  MissionFollowmeTask.msg
  MissionFollowmeTarget.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Activation.srv
  AttitudeControl.srv
  CameraActionControl.srv
  DroneTaskControl.srv
  GimbalAngleControl.srv
  GimbalSpeedControl.srv
  GlobalPositionControl.srv
  LocalPositionControl.srv
  SDKPermissionControl.srv
  VelocityControl.srv
  VersionCheck.srv

  DroneArmControl.srv
  SyncFlagControl.srv
  MessageFrequencyControl.srv
  VirtualRCEnableControl.srv
  VirtualRCDataControl.srv

  MissionStart.srv
  MissionPause.srv
  MissionCancel.srv
  MissionDownload.srv
  MissionWpUpload.srv
  MissionWpSetSpeed.srv
  MissionWpGetSpeed.srv
  MissionHpUpload.srv
  MissionHpSetSpeed.srv
  MissionHpSetRadius.srv
  MissionHpResetYaw.srv
  MissionFmUpload.srv
  MissionFmSetTarget.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  GlobalPositionNavigation.action
  LocalPositionNavigation.action
  WaypointNavigation.action
  DroneTask.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
  actionlib_msgs
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
#  LIBRARIES dji_sdk
  CATKIN_DEPENDS message_runtime actionlib actionlib_msgs geometry_msgs nav_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS})

## Declare a cpp library
# add_library(dji_sdk
#   src/${PROJECT_NAME}/dji_sdk.cpp
# )

## Declare a cpp executable
add_executable(dji_sdk_node
  src/dji_sdk_node.cpp
  src/modules/dji_sdk_node_services.cpp
  src/modules/dji_sdk_node_main.cpp

  include/dji_sdk/DJI_LIB/DJI_Pro_App.cpp
  include/dji_sdk/DJI_LIB/DJI_Pro_Codec.cpp
  include/dji_sdk/DJI_LIB/DJI_Pro_Hw.cpp
  include/dji_sdk/DJI_LIB/DJI_Pro_Link.cpp
  include/dji_sdk/DJI_LIB/DJI_Pro_Rmu.cpp)

add_dependencies(dji_sdk_node dji_sdk_generate_messages_cpp)
target_link_libraries(dji_sdk_node ${catkin_LIBRARIES})
