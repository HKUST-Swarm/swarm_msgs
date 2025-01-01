cmake_minimum_required(VERSION 3.8)
project(swarm_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(swarmtal_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SwarmFused.msg"
  "msg/SwarmFusedRelative.msg"
  "msg/SwarmRemoteCommand.msg"
  "msg/SwarmDroneSourceData.msg"
  "msg/NodeDetectedXyzYaw.msg"
  "msg/NodeFrame.msg"
  "msg/SwarmFrame.msg"
  "msg/NodeDetected.msg"
  "msg/SwarmDetected.msg"
  "msg/SwarmDroneBasecoor.msg"
  "msg/ImageDescriptor.msg"
  "msg/LoopEdge.msg"
  "msg/LoopEdges.msg"
  "msg/Landmark.msg"
  "msg/DroneTraj.msg"
  "msg/VIOFrame.msg"
  "msg/ImageArrayDescriptor.msg"
  "msg/DPGOData.msg"
  "msg/DPGOSignal.msg"
  DEPENDENCIES
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    swarmtal_msgs
    builtin_interfaces
)

# 4. 导出依赖
ament_export_dependencies(rosidl_default_runtime)

# Install the include directory
install(DIRECTORY include/
  DESTINATION include/swarm_msgs/
)
# 6. 结束
ament_package()
