cmake_minimum_required(VERSION 2.8.3)
project(ig_active_reconstruction_msgs)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  message_generation
  geometry_msgs
  sensor_msgs
)
add_message_files(
  FILES
  InformationGain.msg
  InformationGainRetrievalCommand.msg
  InformationGainRetrievalConfig.msg
  MovementCostMsg.msg
  SubWindow.msg
  ViewMsg.msg
  ViewSpaceMsg.msg
)

add_service_files(
  FILES
  DeleteViews.srv
  InformationGainCalculation.srv
  MapMetricCalculation.srv
  MovementCostCalculation.srv
  MoveToOrder.srv
  PclInput.srv
  RetrieveData.srv
  StringList.srv
  ViewRequest.srv
  ViewSpaceRequest.srv
  ViewSpaceUpdate.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)