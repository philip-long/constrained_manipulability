cmake_minimum_required(VERSION 3.5)
project(constrained_manipulability)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(kdl_parser REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_eigen_kdl REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(urdf QUIET)
if(urdf_FOUND)
  add_definitions(-DHAS_URDF)
  include_directories(${urdf_INCLUDE_DIRS})
endif()

find_package(constrained_manipulability_interfaces REQUIRED)
find_package(octomap_filter_interfaces REQUIRED)
find_package(robot_collision_checking REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

find_library(EIGEN_CDDLIB eigen-cddlib)

### Libraries and executables ###

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  rclpy
  kdl_parser
  geometry_msgs
  geometric_shapes
  octomap_msgs
  pcl_ros
  sensor_msgs
  shape_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_eigen_kdl
  tf2_ros
  urdf
  visualization_msgs
  constrained_manipulability_interfaces
  octomap_filter_interfaces
  robot_collision_checking
)

add_library(${PROJECT_NAME} SHARED
  src/constrained_manipulability.cpp
  src/constrained_manipulability_utils.cpp
  src/polytope.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${Eigen3_LIBRARIES}
  ${EIGEN_CDDLIB}
  ${OCTOMAP_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

add_executable(constrained_manipulability_node
  src/constrained_manipulability_main.cpp
)
target_link_libraries(constrained_manipulability_node
  ${PROJECT_NAME}
)
ament_target_dependencies(constrained_manipulability_node
  ${dependencies}
)

### Install ###

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS constrained_manipulability_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# install(PROGRAMS 
#   scripts/cm_client.py
#   scripts/lin_limit_pub.py
#   DESTINATION lib/${PROJECT_NAME}
# )

### Ament stuff ###

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  ${dependencies}
  Eigen3
  EIGEN_CDDLIB
  octomap
)

ament_package()