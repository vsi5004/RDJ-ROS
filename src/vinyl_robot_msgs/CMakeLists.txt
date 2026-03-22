cmake_minimum_required(VERSION 3.8)
project(vinyl_robot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionStatus.msg"
  "msg/AlbumMetadata.msg"
  "action/MoveToPosition.action"
  "action/HomeAll.action"
  "action/Grip.action"
  "action/FlipRecord.action"
  "action/PressPlay.action"
  "action/SetSpeed.action"
  DEPENDENCIES std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
