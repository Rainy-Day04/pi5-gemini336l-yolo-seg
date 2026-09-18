cmake_minimum_required(VERSION 3.8)
project(gemini336l_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Object2D.msg"
  "msg/Object2DArray.msg"
  "msg/Object3D.msg"
  "msg/Object3DArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
