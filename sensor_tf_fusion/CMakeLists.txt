cmake_minimum_required(VERSION 3.8)
project(sensor_tf_fusion)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  sensor_tf_fusion/robot_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
