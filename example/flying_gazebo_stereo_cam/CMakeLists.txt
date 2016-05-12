cmake_minimum_required(VERSION 2.8.3)
project(flying_gazebo_stereo_cam)

list( APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  urdf
  ig_active_reconstruction
  ig_active_reconstruction_msgs
  ig_active_reconstruction_ros
  ig_active_reconstruction_octomap
  movements
  tf
  gazebo_msgs
  geometry_msgs
  sensor_msgs
)

find_package(Eigen REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    urdf
    ig_active_reconstruction
    ig_active_reconstruction_msgs
    ig_active_reconstruction_ros
    ig_active_reconstruction_octomap
    movements
    geometry_msgs
    sensor_msgs
  DEPENDS
    Eigen
    Boost
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

file(GLOB ${PROJECT_NAME}_CODE_BASE
  "src/code_base/*.cpp"
)

add_library(${PROJECT_NAME} STATIC
  ${${PROJECT_NAME}_CODE_BASE}
)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

# Executables...........................................................

add_executable(robot_interface
  src/ros_nodes/robot_interface.cpp
  ${${PROJECT_NAME}_CODE_BASE}
)
target_link_libraries(robot_interface
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_dependencies(robot_interface
 ${catkin_EXPORTED_TARGETS}
)
