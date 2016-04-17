cmake_minimum_required(VERSION 2.8.3)
project(ig_based_active_reconstruction_msgs)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  message_generation
  geometry_msgs
)
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
  SaveViewSpace.srv
  SetScale.srv
  Booleans.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)