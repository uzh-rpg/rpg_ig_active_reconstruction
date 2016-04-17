/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of ig_based_active_reconstruction, a ROS package for...well,

ig_based_active_reconstruction is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
ig_based_active_reconstruction is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with ig_based_active_reconstruction. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ig_based_active_reconstruction/robot_planning_interface.h"


namespace ig_based_active_reconstruction
{

RobotPlanningInterface::RobotPlanningInterface()
{
  
}

boost::shared_ptr<RobotPlanningInterface::PlanningSpaceInitializationInfo::RobotSpaceInfo> RobotPlanningInterface::PlanningSpaceInitializationInfo::getSpecifics()
{
  return robot_specific_info_;
}

ig_based_active_reconstruction_msgs::MovementCostMsg RobotPlanningInterface::MovementCost::toMsg()
{
  ig_based_active_reconstruction_msgs::MovementCostMsg msg;
  msg.cost = cost;
  msg.exception = static_cast<uint32_t>(exception);
  msg.additional_fields_names = additional_field_names;
  msg.additional_fields_values = additional_fields_values;
  return msg;
}

void RobotPlanningInterface::MovementCost::fromMsg( ig_based_active_reconstruction_msgs::MovementCostMsg& _msg )
{
  cost = _msg.cost;
  exception = RobotPlanningInterface::MovementCost::Exception(_msg.exception);
  additional_field_names = _msg.additional_fields_names;
  additional_fields_values = _msg.additional_fields_values;
}

}