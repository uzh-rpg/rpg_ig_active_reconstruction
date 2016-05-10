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

#include "ig_active_reconstruction_ros/views_conversions.hpp"
#include "movements/ros_movements.h"
#include <stdexcept>

namespace ig_active_reconstruction
{
    
namespace ros_conversions
{
  
  ig_active_reconstruction_msgs::ViewMsg viewToMsg( const views::View& view )
  {
    ig_active_reconstruction_msgs::ViewMsg msg;
    msg.pose = movements::toROS( view.pose() );
    msg.source_frame = view.sourceFrame();
    msg.is_bad = view.bad();
    msg.visited = view.timesVisited();
    msg.is_reachable = view.reachable();
    msg.associated_names = view.additionalFieldsNames();
    msg.associated_values = view.additionalFieldsValues();
    msg.index = view.index();
    return msg;
  }
  
  views::View viewFromMsg( ig_active_reconstruction_msgs::ViewMsg& msg )
  {
    views::View view(msg.index);
    view.pose() = movements::fromROS(msg.pose);
    view.sourceFrame() = msg.source_frame;
    view.reachable() = msg.is_reachable;
    view.bad() = msg.is_bad;
    view.timesVisited() = msg.visited;
    view.additionalFieldsNames() = msg.associated_names;
    view.additionalFieldsValues() = msg.associated_values;
    
    return view;
  }
  
  ig_active_reconstruction_msgs::ViewSpaceMsg viewSpaceToMsg( const views::ViewSpace& view_space )
  {
    ig_active_reconstruction_msgs::ViewSpaceMsg msg;
    for( views::View const & view: view_space )
    {
      msg.views.push_back( viewToMsg(view) );
    }
    
    return msg;
  }
  
  views::ViewSpace viewSpaceFromMsg( ig_active_reconstruction_msgs::ViewSpaceMsg& msg )
  {
    views::ViewSpace view_space;
    std::cout<<"\n\n size: "<<msg.views.size();
    
    for( auto& view: msg.views )
    {
      view_space.push_back( viewFromMsg(view) );
    }
    return view_space;
  }
  
  views::CommunicationInterface::ViewSpaceStatus viewSpaceStatusFromMsg( int& msg )
  {
    switch(msg)
    {
      case 0: return views::CommunicationInterface::ViewSpaceStatus::OK;
      case 1: return views::CommunicationInterface::ViewSpaceStatus::BAD;
      case 2: return views::CommunicationInterface::ViewSpaceStatus::NONE_AVAILABLE;
      default: throw std::invalid_argument("ig_active_reconstruction::ros_conversions::viewSpaceStatusFromMsg:: Invalid msg received.");
    };
  }
  
  int viewSpaceStatusToMsg( views::CommunicationInterface::ViewSpaceStatus& status )
  {
    switch(status)
    {
      case views::CommunicationInterface::ViewSpaceStatus::OK: return 0;
      case views::CommunicationInterface::ViewSpaceStatus::BAD: return 1;
      case views::CommunicationInterface::ViewSpaceStatus::NONE_AVAILABLE: return 2;
      default: throw std::invalid_argument("ig_active_reconstruction::ros_conversions::viewSpaceStatusToMsg:: Invalid status received.");
    };
  }
  
  views::CommunicationInterface::ViewSpaceUpdateResult viewSpaceUpdateResultFromMsg(int& msg)
  {
    switch(msg)
    {
      case 0: return views::CommunicationInterface::ViewSpaceUpdateResult::SUCCEEDED;
      case 1: return views::CommunicationInterface::ViewSpaceUpdateResult::FAILED;
      case 2: return views::CommunicationInterface::ViewSpaceUpdateResult::NOT_AVAILABLE;
    };
  }
  
  int viewSpaceUpdateResultToMsg(views::CommunicationInterface::ViewSpaceUpdateResult& res)
  {
    switch(res)
    {
      case views::CommunicationInterface::ViewSpaceUpdateResult::SUCCEEDED: return 0;
      case views::CommunicationInterface::ViewSpaceUpdateResult::FAILED: return 1;
      case views::CommunicationInterface::ViewSpaceUpdateResult::NOT_AVAILABLE: return 2;
    };
  }
  
}

}