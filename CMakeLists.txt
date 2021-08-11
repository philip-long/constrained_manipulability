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
  robot_collision_checking
  roscpp
  rospy
  message_generation
)


find_library(EIGEN_CDDLIB_LIB eigen-cddlib)


set(SNOPT_LIB $ENV{SNOPT_DIR}/lib/libsnopt7_cpp.so) # SNOPT INCLUDE


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   ObjectDistances.msg
   PolytopeMesh.msg
   PolytopeVolume.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  shape_msgs
)


catkin_package(
  INCLUDE_DIRS include
 LIBRARIES constrained_manipulability
 CATKIN_DEPENDS eigen_conversions geometric_shapes kdl_parser pcl_ros robot_collision_checking roscpp rospy
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
  $ENV{SNOPT_DIR}/include # SNOPT INCLUDE
)

add_library(constrained_manipulability
            src/constrained_manipulability.cpp
            include/${PROJECT_NAME}/constrained_manipulability.h)
target_link_libraries(constrained_manipulability ${catkin_LIBRARIES} ${EIGEN_CDDLIB_LIB} ${fcl_interface_LIBRARIES})

## Add cmake target dependencies of the library for message generation code necessary before library compilation
add_dependencies(constrained_manipulability ${constrained_manipulability_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(abstract_robot_test test/abstract_robot_test_node.cpp)
target_link_libraries(abstract_robot_test ${catkin_LIBRARIES} constrained_manipulability)

add_executable(ik_example test/ik_example.cpp)
target_link_libraries(ik_example ${SNOPT_LIB} ${catkin_LIBRARIES} constrained_manipulability)

add_executable(shrinking_polytope test/shrinking_polytope.cpp)
target_link_libraries(shrinking_polytope ${catkin_LIBRARIES} constrained_manipulability)

add_executable(slicing_polytope test/slicing_polytope.cpp)
target_link_libraries(slicing_polytope ${catkin_LIBRARIES} constrained_manipulability)