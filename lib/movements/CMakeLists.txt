cmake_minimum_required(VERSION 2.8.3)
project(movements)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
set(movements_INCLUDE_DIRS
${CMAKE_CURRENT_SOURCE_DIR}/include
${catkin_INCLUDE_DIRS}
${Eigen_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
PARENT_SCOPE
)

add_library(movements
  src/combined_kinematic_movement_description.cpp
  src/combined_relative_movement.cpp
  src/geometry_pose.cpp
  src/kinematic_movement_description.cpp
  src/relative_movement.cpp
  src/ros_movements.cpp
  src/translation.cpp
  src/in_out_spiral.cpp
  src/linear_movement.cpp
)