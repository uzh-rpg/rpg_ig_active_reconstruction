cmake_minimum_required(VERSION 2.8.3)
project(flying_gazebo_stereo_cam)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  urdf
  ig_active_reconstruction
  ig_active_reconstruction_msgs
  movements
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    urdf
    ig_active_reconstruction
    ig_active_reconstruction_msgs
    movements
  DEPENDS
    Eigen
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

file(GLOB ${PROJECT_NAME}_CODE_BASE
  "src/code_base/*.cpp"
)

add_library(${PROJECT_NAME} STATIC
  ${${PROJECT_NAME}_CODE_BASE}
)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)