cmake_minimum_required(VERSION 2.8.3)
project(ig_active_reconstruction_octomap)
set(CMAKE_BUILD_TYPE Release)

## c++11 is preferred but ROS is built with c++03 and the PCL binaries are not compatible (boost)
##list( APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  movements
  ig_active_reconstruction
  roscpp
  ig_active_reconstruction_ros
  ig_active_reconstruction_msgs
  sensor_msgs
  tf
  pcl_ros
  visualization_msgs
  geometry_msgs
  std_msgs
)

find_package(octomap REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    movements
    ig_active_reconstruction
    roscpp
    ig_active_reconstruction_ros
    ig_active_reconstruction_msgs
    sensor_msgs
    tf
    pcl_ros
    visualization_msgs
    geometry_msgs
    std_msgs
  DEPENDS
    PCL
    Eigen
    octomap
    Boost
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

file(GLOB ${PROJECT_NAME}_CODE_BASE
  "src/code_base/*.cpp"
  "src/code_base/ig/*.cpp"
)

set(${PROJECT_NAME}_LIBRARIES
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${Boost_LIBRARIES}
)


add_library(${PROJECT_NAME} STATIC
  ${${PROJECT_NAME}_CODE_BASE}
)

target_link_libraries(${PROJECT_NAME}
   ${${PROJECT_NAME}_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} 
 ${catkin_EXPORTED_TARGETS}
)

# Executables...........................................................

add_executable(octomap_world_representation
  src/ros_nodes/octomap_world_representation.cpp
  ${${PROJECT_NAME}_CODE_BASE}
)
target_link_libraries(octomap_world_representation
   ${${PROJECT_NAME}_LIBRARIES}
)
add_dependencies(octomap_world_representation
 ${catkin_EXPORTED_TARGETS}
)
