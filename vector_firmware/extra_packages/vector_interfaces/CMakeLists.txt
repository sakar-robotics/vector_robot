cmake_minimum_required(VERSION 3.8)
project(vector_interfaces)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/EncoderTicks.msg
  msg/MotorTicksSec.msg
  DEPENDENCIES
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
