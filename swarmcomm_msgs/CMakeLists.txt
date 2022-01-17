cmake_minimum_required(VERSION 2.8.3)
project(swarmcomm_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    remote_uwb_info.msg
    incoming_broadcast_data.msg
    data_buffer.msg
    drone_network_status.msg
    swarm_network_status.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  nav_msgs 
  geometry_msgs
  std_msgs
  sensor_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES swarm_msgs
#  CATKIN_DEPENDS message_generation message_runtime nav_msgs odometry_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

