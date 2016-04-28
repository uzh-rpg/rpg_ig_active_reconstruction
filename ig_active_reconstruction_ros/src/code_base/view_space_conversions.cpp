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

#include "ig_active_reconstruction_ros/view_space_conversions.hpp"
#include "ig_active_reconstruction_ros/view_conversions.hpp"


namespace ig_active_reconstruction
{
  
namespace views
{
  ig_active_reconstruction_msgs::ViewSpaceMsg viewSpaceToMsg( ViewSpace& view_space )
  {
    ig_active_reconstruction_msgs::ViewSpaceMsg msg;
    for( View& view: view_space )
    {
      msg.views.push_back( viewToMsg(view) );
    }
  }
  
  ViewSpace viewSpaceFromMsg( ig_active_reconstruction_msgs::ViewSpaceMsg& msg )
  {
    ViewSpace view_space;
    
    for( auto& view: msg.views )
    {
      view_space.push_back( viewFromMsg(view) );
    }
  }
}

}