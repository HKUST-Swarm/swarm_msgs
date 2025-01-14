cmake_minimum_required(VERSION 3.8)
project(swarmtal_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DronePosCtrlCmd.msg"
  "msg/DronePosControlState.msg"
  "msg/DroneCommanderCoreState.msg"
  "msg/DroneCommanderState.msg"
  "msg/DroneOnboardCommand.msg"
  DEPENDENCIES
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
