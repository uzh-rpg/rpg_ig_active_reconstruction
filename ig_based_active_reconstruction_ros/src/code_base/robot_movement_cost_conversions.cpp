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

#include "ig_based_active_reconstruction_ros/robot_movement_cost_conversions.hpp"

namespace ig_based_active_reconstruction
{
    
namespace robot
{
  ig_based_active_reconstruction_msgs::MovementCostMsg movementCostToMsg(const MovementCost& cost)
  {
    ig_based_active_reconstruction_msgs::MovementCostMsg msg;
    msg.cost = cost.cost;
    msg.exception = static_cast<uint32_t>(cost.exception);
    msg.additional_fields_names = cost.additional_field_names;
    msg.additional_fields_values = cost.additional_fields_values;
    return msg;
  }

  MovementCost movementCostFromMsg( ig_based_active_reconstruction_msgs::MovementCostMsg& _msg )
  {
    MovementCost cost;
    cost.cost = _msg.cost;
    cost.exception = MovementCost::Exception(_msg.exception);
    cost.additional_field_names = _msg.additional_fields_names;
    cost.additional_fields_values = _msg.additional_fields_values;
    return cost;
  }
}

}