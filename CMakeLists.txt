cmake_minimum_required(VERSION 2.8.3)
project(dense_reconstruction)

list( APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  geometry_msgs
  roscpp
  tf
  moveit_core
  moveit_ros_planning_interface
  control_msgs
  svo_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system ) #accumulators bind ref)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

list(APPEND USER_LIBRARIES movements)
add_subdirectory(lib)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs cv_bridge moveit_core moveit_ros_planning_interface control_msgs tf gazebo_msgs
  DEPENDS opencv opencv2 Eigen
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${movements_INCLUDE_DIRS}
)


add_executable(youbot_control
  src/dense_reconstruction/ros_nodes/develop.cpp
  src/dense_reconstruction/code_base/youbot_reconstruction_controller.cpp
  src/utils/ros_eigen.cpp
  ${movement_sources}
)

target_link_libraries(youbot_control
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  movements
)

add_dependencies(youbot_control
 ${catkin_EXPORTED_TARGETS}
)


add_executable(remode_feeder
  src/dense_reconstruction/code_base/remode_feeder.cpp
  src/dense_reconstruction/ros_nodes/feeder_node.cpp
  src/utils/ros_eigen.cpp
  ${movement_sources}
)

target_link_libraries(remode_feeder
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  movements
)

add_dependencies(remode_feeder
 ${catkin_EXPORTED_TARGETS}
)