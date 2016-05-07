/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of ig_active_reconstruction, a ROS package for...well,

ig_active_reconstruction is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
ig_active_reconstruction is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with ig_active_reconstruction. If not, see <http://www.gnu.org/licenses/>.
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
    ig_active_reconstruction_msgs::ViewMsg viewToMsg( views::View& view );

    /*! Constructs a new view from a view message.
      */
    views::View viewFromMsg( ig_active_reconstruction_msgs::ViewMsg& msg );
    
    /** Creates a view space msg with the content of the view space
    */
    ig_active_reconstruction_msgs::ViewSpaceMsg viewSpaceToMsg( views::ViewSpace& view_space );
    
    /** Construct view space from message */
    views::ViewSpace viewSpaceFromMsg( ig_active_reconstruction_msgs::ViewSpaceMsg& msg );
    
  
    views::CommunicationInterface::ViewSpaceStatus viewSpaceStatusFromMsg( int& msg );
    int viewSpaceStatusToMsg( views::CommunicationInterface::ViewSpaceStatus& status );
    
    views::CommunicationInterface::ViewSpaceUpdateResult viewSpaceUpdateResultFromMsg(int& msg);
    int viewSpaceUpdateResultToMsg(views::CommunicationInterface::ViewSpaceUpdateResult& res);
    
}

}