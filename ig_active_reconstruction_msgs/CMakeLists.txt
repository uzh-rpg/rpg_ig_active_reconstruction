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
  SubWindow.msg
  InformationGain.msg
  InformationGainRetrievalConfig.msg
  InformationGainRetrievalCommand.msg
  ViewMsg.msg
  ViewSpaceMsg.msg
  MovementCostMsg.msg
)

add_service_files(
  FILES
  PoseSetter.srv
  IsOk.srv
  InformationGainCalculation.srv
  PlanningSpaceInitializationInfoMsg.srv
  ViewSpaceRequest.srv
  ViewRequest.srv
  RetrieveData.srv
  MovementCostCalculation.srv
  MoveToOrder.srv
  SaveViewSpace.srv
  SetScale.srv
  Booleans.srv
  DeleteViews.srv
  ViewSpaceUpdate.srv
  StringList.srv
  MapMetricCalculation.srv
  PclInput.srv
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