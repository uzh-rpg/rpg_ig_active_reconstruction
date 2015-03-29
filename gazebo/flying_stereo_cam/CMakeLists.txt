cmake_minimum_required(VERSION 2.8.3)
project(flying_stereo_cam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp urdf
)
