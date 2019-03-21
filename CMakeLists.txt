cmake_minimum_required(VERSION 2.8.3)
project(constrained_manipulability)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometric_shapes
  kdl_parser
  pcl_ros
  robot_collision_checking
  roscpp
  rospy
  rviz_display
  message_generation
)


find_library(EIGEN_CDDLIB_LIB eigen-cddlib)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   PolytopeVolume.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES constrained_manipulability
#  CATKIN_DEPENDS eigen_conversions geometric_shapes kdl_parser pcl_ros robot_collision_checking roscpp rospy rviz_display
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

add_library(constrained_manipulability
            src/constrained_manipulability.cpp
            include/${PROJECT_NAME}/constrained_manipulability.h)
target_link_libraries(constrained_manipulability ${catkin_LIBRARIES} ${EIGEN_CDDLIB_LIB} ${fcl_interface_LIBRARIES} ${rviz_display_LIBRARIES})

add_executable(abstract_robot_test test/abstract_robot_test_node.cpp)
target_link_libraries(abstract_robot_test ${catkin_LIBRARIES} constrained_manipulability)

