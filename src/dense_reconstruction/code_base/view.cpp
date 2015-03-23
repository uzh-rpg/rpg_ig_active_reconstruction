/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of dense_reconstruction, a ROS package for...well,

dense_reconstruction is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
dense_reconstruction is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with dense_reconstruction. If not, see <http://www.gnu.org/licenses/>.
*/

#include "dense_reconstruction/view.h"
#include <movements/ros_movements.h>

namespace dense_reconstruction
{

View::View():
  is_reachable_(true),
  is_bad_(false),
  visited_(0)
{
  
}

View::View( std::string _source_frame ):
source_frame_(_source_frame),
  is_reachable_(true),
  is_bad_(false),
  visited_(0)
{
  
}

View::View( ViewMsg& _msg )
{
  pose_ = movements::fromROS(_msg.pose);
  source_frame_ = _msg.source_frame;
  is_reachable_ = _msg.is_reachable;
  is_bad_ = _msg.is_bad;
  visited_ = _msg.visited;
  additional_fields_names_ = _msg.associated_names;
  additional_fields_values_ = _msg.associated_values;
}

ViewMsg View::toMsg()
{
  ViewMsg msg;
  msg.pose = movements::toROS(pose_);
  msg.source_frame = source_frame_;
  msg.is_bad = is_bad_;
  msg.visited = visited_;
  msg.associated_names = additional_fields_names_;
  msg.associated_values = additional_fields_values_;
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

boost::shared_ptr<dense_reconstruction::View::ViewInfo>& View::associatedData()
{
  return associated_data_;
}


}

std::ostream& operator<<(std::ostream& _out, dense_reconstruction::View& _view )
{
  _out<<"Pose:\n";
  _out<<"  position: \n";
  _out<<"    x: "<<_view.pose().position.x()<<"\n";
  _out<<"    y: "<<_view.pose().position.y()<<"\n";
  _out<<"    z: "<<_view.pose().position.z()<<"\n";
  _out<<"  orientation: \n";
  _out<<"    x: "<<_view.pose().orientation.x()<<"\n";
  _out<<"    y: "<<_view.pose().orientation.y()<<"\n";
  _out<<"    z: "<<_view.pose().orientation.z()<<"\n";
  _out<<"    w: "<<_view.pose().orientation.w()<<"\n";
  return _out;
}