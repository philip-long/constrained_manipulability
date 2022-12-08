cmake_minimum_required(VERSION 3.1)
project(constrained_manipulability)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometric_shapes
  kdl_parser
  pcl_ros
  sensor_msgs
  robot_collision_checking
  roscpp
  rospy
  message_generation
  octomap_filter
)


find_library(EIGEN_CDDLIB_LIB eigen-cddlib)


add_service_files(
  FILES
  GetPolytopeConstraints.srv
  GetJacobianMatrix.srv
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   ObjectDistances.msg
   PolytopeMesh.msg
   PolytopeVolume.msg
   PolytopeHyperplanes.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  shape_msgs
  sensor_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES constrained_manipulability
  CATKIN_DEPENDS eigen_conversions geometric_shapes kdl_parser pcl_ros robot_collision_checking roscpp rospy message_runtime octomap_filter octomap_server
#  DEPENDS system_lib
)

###########
## Build ##
###########
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
            src/constrained_manipulability.cpp
            src/polytope.cpp
            src/static_constrained_manipulability.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN_CDDLIB_LIB} ${fcl_interface_LIBRARIES})

add_executable(abstract_robot_test nodes/abstract_robot_test.cpp)
target_link_libraries(abstract_robot_test ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(constrained_manipulability_server nodes/constrained_manipulability_server.cpp)
target_link_libraries(constrained_manipulability_server ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(shrinking_polytope_test nodes/shrinking_polytope_test.cpp)
target_link_libraries(shrinking_polytope_test ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(slicing_polytope_test nodes/slicing_polytope_test.cpp)
target_link_libraries(slicing_polytope_test ${catkin_LIBRARIES} ${PROJECT_NAME})