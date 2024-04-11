cmake_minimum_required(VERSION 3.5)
project(constrained_manipulability_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/Matrix.msg
  msg/ObjectDistances.msg
  msg/PolytopeHyperplanes.msg
  msg/PolytopeMesh.msg
  msg/PolytopeVolume.msg
)

set(srv_files
  srv/GetJacobianMatrix.srv
  srv/GetPolytopeConstraints.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces sensor_msgs shape_msgs std_msgs
)

install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}/)

ament_export_dependencies(rosidl_default_runtime)

ament_package()