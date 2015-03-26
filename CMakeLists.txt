cmake_minimum_required(VERSION 2.8.3)
project(dense_reconstruction)

list( APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  angles
  sensor_msgs
  cv_bridge
  geometry_msgs
  roscpp
  tf
  moveit_core
  moveit_ros_planning_interface
  control_msgs
  svo_msgs
  message_generation
  std_msgs
  octomap_server
  octomap_msgs
  svo_srv
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system ) #accumulators bind ref)

find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)

find_package(octomap REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

list(APPEND USER_LIBRARIES movements)
add_subdirectory(lib)

find_package(Eigen REQUIRED)

add_message_files(
  FILES
  ViewInformation.msg
  ViewInformationRequest.msg
  ViewMsg.msg
  ViewSpaceMsg.msg
  MovementCostMsg.msg
)

add_service_files(
  FILES
  PoseSetter.srv
  IsOk.srv
  ViewInformationReturn.srv
  PlanningSpaceInitializationInfoMsg.srv
  FeasibleViewSpaceRequest.srv
  ViewRequest.srv
  RetrieveData.srv
  MovementCostCalculation.srv
  MoveToOrder.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs cv_bridge moveit_core moveit_ros_planning_interface control_msgs tf gazebo_msgs message_runtime std_msgs angles octomap_server octomap_msgs svo_srv
  DEPENDS opencv opencv2 Eigen
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${movements_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

set(DENSE_RECONSTRUCTION_BASE_SRC
  src/dense_reconstruction/code_base/robot_planning_interface.cpp
  src/dense_reconstruction/code_base/view.cpp
  src/dense_reconstruction/code_base/view_planner.cpp
  src/dense_reconstruction/code_base/view_space.cpp
  src/utils/math.cpp
)


add_executable(youbot_interface
  src/dense_reconstruction/ros_nodes/youbot_planner_node.cpp
  src/dense_reconstruction/code_base/youbot_planning.cpp
  src/dense_reconstruction/code_base/remode_data_retriever.cpp
  src/utils/ros_eigen.cpp
  ${movement_sources}
  ${DENSE_RECONSTRUCTION_BASE_SRC}
)

target_link_libraries(youbot_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  movements
)

add_dependencies(youbot_interface
 ${catkin_EXPORTED_TARGETS}
)


add_executable(dense_reconstruction_control
  src/dense_reconstruction/ros_nodes/dense_reconstruction_control.cpp
  ${movement_sources}
  ${DENSE_RECONSTRUCTION_BASE_SRC}
)

target_link_libraries(dense_reconstruction_control
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  movements
)

add_dependencies(dense_reconstruction_control
 ${catkin_EXPORTED_TARGETS}
)


add_executable(develop
  src/dense_reconstruction/ros_nodes/develop.cpp
  src/dense_reconstruction/code_base/youbot_planning.cpp
  src/dense_reconstruction/code_base/remode_data_retriever.cpp
  src/utils/ros_eigen.cpp
  ${movement_sources}
  ${DENSE_RECONSTRUCTION_BASE_SRC}
)

target_link_libraries(develop
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  movements
)

add_dependencies(develop
 ${catkin_EXPORTED_TARGETS}
)


add_executable(view_planner
  src/dense_reconstruction/ros_nodes/view_planner_node.cpp
  src/utils/ros_eigen.cpp
  ${movement_sources}
  ${DENSE_RECONSTRUCTION_BASE_SRC}
)

target_link_libraries(view_planner
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  movements
)

add_dependencies(view_planner
 ${catkin_EXPORTED_TARGETS}
)


add_executable(youbot_fixed_waypoint_reconstruction
  src/dense_reconstruction/ros_nodes/youbot_fixed_waypoint_reconstruction_node.cpp
  src/dense_reconstruction/code_base/youbot_fixed_waypoint_reconstruction_controller.cpp
  src/utils/ros_eigen.cpp
  ${movement_sources}
)

target_link_libraries(youbot_fixed_waypoint_reconstruction
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  movements
)

add_dependencies(youbot_fixed_waypoint_reconstruction
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


add_executable(tf_linker
  src/dense_reconstruction/ros_nodes/tf_linker.cpp
  ${movement_sources}
)

target_link_libraries(tf_linker
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(tf_linker
 ${catkin_EXPORTED_TARGETS}
 ${PROJECT_NAME}_generate_messages_cpp
)