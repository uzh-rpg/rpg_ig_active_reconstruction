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

#include "ig_based_active_reconstruction/view.hpp"
#include <movements/ros_movements.h>

namespace ig_based_active_reconstruction
{

View::View():
  is_reachable_(true),
  is_bad_(false),
  visited_(0)
{
  
}

View::View( std::string source_frame ):
source_frame_(source_frame),
  is_reachable_(true),
  is_bad_(false),
  visited_(0)
{
  
}

View::View( ig_based_active_reconstruction_msgs::ViewMsg& msg )
{
  pose_ = movements::fromROS(msg.pose);
  source_frame_ = msg.source_frame;
  is_reachable_ = msg.is_reachable;
  is_bad_ = msg.is_bad;
  visited_ = msg.visited;
  additional_fields_names_ = msg.associated_names;
  additional_fields_values_ = msg.associated_values;
  index = msg.index;
}

ig_based_active_reconstruction_msgs::ViewMsg View::toMsg()
{
  ig_based_active_reconstruction_msgs::ViewMsg msg;
  msg.pose = movements::toROS(pose_);
  msg.source_frame = source_frame_;
  msg.is_bad = is_bad_;
  msg.visited = visited_;
  msg.is_reachable = is_reachable_;
  msg.associated_names = additional_fields_names_;
  msg.associated_values = additional_fields_values_;
  msg.index = index;
  return msg;
}

movements::Pose& View::pose()
{
  return pose_;
}

std::string& View::sourceFrame()
{
  return source_frame_;
}

bool& View::reachable()
{
  return is_reachable_;
}

unsigned int& View::timesVisited()
{
  return visited_;
}

bool& View::bad()
{
  return is_bad_;
}

boost::shared_ptr<ig_based_active_reconstruction::View::ViewInfo>& View::associatedData()
{
  return associated_data_;
}


}

std::ostream& operator<<(std::ostream& _out, ig_based_active_reconstruction::View& view )
{
  _out<<"Pose:\n";
  _out<<"  position: \n";
  _out<<"    x: "<<view.pose().position.x()<<"\n";
  _out<<"    y: "<<view.pose().position.y()<<"\n";
  _out<<"    z: "<<view.pose().position.z()<<"\n";
  _out<<"  orientation: \n";
  _out<<"    x: "<<view.pose().orientation.x()<<"\n";
  _out<<"    y: "<<view.pose().orientation.y()<<"\n";
  _out<<"    z: "<<view.pose().orientation.z()<<"\n";
  _out<<"    w: "<<view.pose().orientation.w()<<"\n";
  return _out;
}