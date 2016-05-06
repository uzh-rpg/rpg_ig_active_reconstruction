cmake_minimum_required(VERSION 2.8.3)
project(ig_active_reconstruction)

list( APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  movements
)

find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    movements
  DEPENDS
    PCL
    Eigen
    octomap
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

file(GLOB ${PROJECT_NAME}_CODE_BASE
  "src/code_base/*.cpp"
  "src/code_base/ig/*.cpp"
)

add_library(${PROJECT_NAME} STATIC
  ${${PROJECT_NAME}_CODE_BASE}
)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} STATIC
 ${catkin_EXPORTED_TARGETS}
)