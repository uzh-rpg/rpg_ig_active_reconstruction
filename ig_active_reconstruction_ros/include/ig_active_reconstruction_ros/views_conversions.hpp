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

#include "ig_active_reconstruction_msgs/ViewMsg.h"
#include "ig_active_reconstruction_msgs/ViewSpaceMsg.h"

#include "ig_active_reconstruction/view.hpp"
#include "ig_active_reconstruction/view_space.hpp"

#include "ig_active_reconstruction/views_communication_interface.hpp"

namespace ig_active_reconstruction
{
    
namespace ros_conversions
{
  
    /*! Converts a view to a view message.
      */
    ig_active_reconstruction_msgs::ViewMsg viewToMsg( const views::View& view );

    /*! Constructs a new view from a view message.
      */
    views::View viewFromMsg( ig_active_reconstruction_msgs::ViewMsg& msg );
    
    /** Creates a view space msg with the content of the view space
    */
    ig_active_reconstruction_msgs::ViewSpaceMsg viewSpaceToMsg( const views::ViewSpace& view_space );
    
    /** Construct view space from message */
    views::ViewSpace viewSpaceFromMsg( ig_active_reconstruction_msgs::ViewSpaceMsg& msg );
    
  
    views::CommunicationInterface::ViewSpaceStatus viewSpaceStatusFromMsg( int& msg );
    int viewSpaceStatusToMsg( views::CommunicationInterface::ViewSpaceStatus& status );
    
    views::CommunicationInterface::ViewSpaceUpdateResult viewSpaceUpdateResultFromMsg(int& msg);
    int viewSpaceUpdateResultToMsg(views::CommunicationInterface::ViewSpaceUpdateResult& res);
    
}

}