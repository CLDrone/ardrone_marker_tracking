cmake_minimum_required(VERSION 2.8.3)
project(velocity_marker_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mavros
  mavros_extras
  geometry_msgs
  tf2_ros
  eigen_conversions
  control_toolbox
  angles
)

find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

include(EnableCXX11)


catkin_package(
)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include
)

add_library(
  PIDController
  src/PIDController.cpp
)

## Declare a C++ executable
add_executable(velocity_marker_tracker_node src/velocity_marker_tracker_node.cpp)

add_dependencies(velocity_marker_tracker_node
  PIDController
)

## Specify libraries to link a library or executable target against
target_link_libraries(velocity_marker_tracker_node
   PIDController
   ${catkin_LIBRARIES}
   ${Boost_INCLUDE_DIRS}
)

