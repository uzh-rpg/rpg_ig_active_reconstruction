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

movements::Pose& View::pose()
{
  return pose_;
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

boost::shared_ptr<dense_reconstruction::ViewInfo> View::associatedData()
{
  return associated_data_;
}


}