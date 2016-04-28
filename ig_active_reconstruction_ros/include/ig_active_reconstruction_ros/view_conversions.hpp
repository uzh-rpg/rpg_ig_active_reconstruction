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
#include "ig_active_reconstruction/view.hpp"

namespace ig_active_reconstruction
{
namespace views
{
  
  /*! Converts a view to a view message.
   */
  ig_active_reconstruction_msgs::ViewMsg viewToMsg( View& view );
  
  /*! Constructs a new view from a view message.
   */
  View viewFromMsg( ig_active_reconstruction_msgs::ViewMsg& msg );
  
}
}