/* Copyright (c) 2016, Stefan Isler, islerstefan@bluewin.ch
 * (ETH Zurich / Robotics and Perception Group, University of Zurich, Switzerland)
 *
 * This file is part of ig_active_reconstruction, software for information gain based, active reconstruction.
 *
 * ig_active_reconstruction is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * ig_active_reconstruction is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * Please refer to the GNU Lesser General Public License for details on the license,
 * on <http://www.gnu.org/licenses/>.
*/

#include "ig_active_reconstruction_ros/robot_conversions.hpp"
#include "movements/ros_movements.h"
#include <stdexcept>

namespace ig_active_reconstruction
{
    
namespace ros_conversions
{
  ig_active_reconstruction_msgs::MovementCostMsg movementCostToMsg(const robot::MovementCost& cost)
  {
    ig_active_reconstruction_msgs::MovementCostMsg msg;
    msg.cost = cost.cost;
    msg.exception = static_cast<uint32_t>(cost.exception);
    msg.additional_fields_names = cost.additional_field_names;
    msg.additional_fields_values = cost.additional_fields_values;
    return msg;
  }

  robot::MovementCost movementCostFromMsg( ig_active_reconstruction_msgs::MovementCostMsg& _msg )
  {
    robot::MovementCost cost;
    cost.cost = _msg.cost;
    cost.exception = robot::MovementCost::Exception(_msg.exception);
    cost.additional_field_names = _msg.additional_fields_names;
    cost.additional_fields_values = _msg.additional_fields_values;
    return cost;
  }
  
  robot::CommunicationInterface::ReceptionInfo robotReceptionInfoFromMsg( int& receive_info )
  {
    if( receive_info==0 )
      return robot::CommunicationInterface::ReceptionInfo::SUCCEEDED;
    else if( receive_info==1 )
      return robot::CommunicationInterface::ReceptionInfo::FAILED;
  }
  
  int robotReceptionInfoToMsg( robot::CommunicationInterface::ReceptionInfo& info )
  {
    if( info==robot::CommunicationInterface::ReceptionInfo::SUCCEEDED )
      return 0;
    else
      return 1;
  }
}

}