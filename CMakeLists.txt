cmake_minimum_required(VERSION 2.8.3)
project(dense_reconstruction)

list( APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  geometry_msgs
  roscpp
  tf2_msgs
  tf2_ros
  moveit_ros_planning_interface
  control_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system ) #accumulators bind ref)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs tf2_msgs tf2_ros sensor_msgs cv_bridge moveit_ros_planning_interface control_msgs message_runtime
  DEPENDS opencv opencv2 Eigen
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(youbot_control
  src/develop.cpp
  src/youbot_reconstruction_controller.cpp
)

target_link_libraries(youbot_control
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(youbot_control
 ${catkin_EXPORTED_TARGETS}
)