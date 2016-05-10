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

#pragma once

#include "ig_active_reconstruction_msgs/MovementCostMsg.h"

#include "ig_active_reconstruction/robot_movement_cost.hpp"

#include "ig_active_reconstruction/robot_communication_interface.hpp"

namespace ig_active_reconstruction
{
    
namespace ros_conversions
{
  
    /*! Converts movement cost object to a ros cost message
     * @param cost Cost object from which a cost message shall be created.
    */
    ig_active_reconstruction_msgs::MovementCostMsg movementCostToMsg( const robot::MovementCost& cost );
  
    /*!
    * loads from msg
    */
    robot::MovementCost movementCostFromMsg( ig_active_reconstruction_msgs::MovementCostMsg& _msg );
    
    robot::CommunicationInterface::ReceptionInfo robotReceptionInfoFromMsg( int& receive_info );
    
    int robotReceptionInfoToMsg( robot::CommunicationInterface::ReceptionInfo& info );
  
}

}