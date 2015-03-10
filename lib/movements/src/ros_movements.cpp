/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of movements, a library for representations and calculations of movements in space,

movements is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
movements is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with movements. If not, see <http://www.gnu.org/licenses/>.
*/

#include "movements/ros_movements.h"
#include <boost/foreach.hpp>

namespace movements
{


geometry_msgs::Pose toROS( movements::Pose _pose )
{
  geometry_msgs::Pose pose;
  
  pose.position.x = _pose.position(0);
  pose.position.y = _pose.position(1);
  pose.position.z = _pose.position(2);
  
  pose.orientation.x = _pose.orientation.x();
  pose.orientation.y = _pose.orientation.y();
  pose.orientation.z = _pose.orientation.z();
  pose.orientation.w = _pose.orientation.w();
  
  return pose;
}


movements::Pose fromROS( geometry_msgs::Pose _pose )
{
  movements::Pose pose;
  
  pose.position(0) = _pose.position.x;
  pose.position(1) = _pose.position.y;
  pose.position(2) = _pose.position.z;
  
  pose.orientation.x() = _pose.orientation.x;
  pose.orientation.y() = _pose.orientation.y;
  pose.orientation.z() = _pose.orientation.z;
  pose.orientation.w() = _pose.orientation.w;
  
  return pose;
}

std::vector<geometry_msgs::Pose> toROS( movements::PoseVector _to_convert )
{
  std::vector<geometry_msgs::Pose> out;
  BOOST_FOREACH( auto pose,  _to_convert )
  {
    out.push_back( toROS(pose) );
  }
  return out;
}

movements::PoseVector fromROS( std::vector<geometry_msgs::Pose> _to_convert )
{
  movements::PoseVector out;
  BOOST_FOREACH( auto pose,  _to_convert )
  {
    out.push_back( fromROS(pose) );
  }
  return out;
}

}