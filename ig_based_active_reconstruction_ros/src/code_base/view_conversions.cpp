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

#include "ig_based_active_reconstruction_ros/view_conversions.hpp"
#include <movements/ros_movements.h>


namespace ig_based_active_reconstruction
{
  
namespace views
{
  
  ig_based_active_reconstruction_msgs::ViewMsg viewToMsg( View& view )
  {
    ig_based_active_reconstruction_msgs::ViewMsg msg;
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
  
  View viewFromMsg( ig_based_active_reconstruction_msgs::ViewMsg& msg )
  {
    View view(msg.index);
    view.pose() = movements::fromROS(msg.pose);
    view.sourceFrame() = msg.source_frame;
    view.reachable() = msg.is_reachable;
    view.bad() = msg.is_bad;
    view.timesVisited() = msg.visited;
    view.additionalFieldsNames() = msg.associated_names;
    view.additionalFieldsValues() = msg.associated_values;
    
    return view;
  }
  
}

}