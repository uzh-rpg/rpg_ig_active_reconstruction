cmake_minimum_required(VERSION 2.8.3)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
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
)