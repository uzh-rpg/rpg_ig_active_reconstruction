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

#include "ig_active_reconstruction/view.hpp"

#include <limits>
#include <iostream>


namespace ig_active_reconstruction
{
  
namespace views
{

View::IdType View::runningIndex_ = 0;

View::View():
  index_(runningIndex_++),
  is_reachable_(true),
  is_bad_(false),
  visited_(0)
{
  if( runningIndex_==std::numeric_limits<IdType>::max() )
    std::cerr<<"Attention::View::index_ is about to overflow! (Next: "<<runningIndex_<<", and the one after: "<<runningIndex_+1<<".";
  
  
}

View::View( std::string source_frame )
  : index_(runningIndex_++)
  , source_frame_(source_frame)
  , is_reachable_(true)
  , is_bad_(false)
  , non_viewspace_(false)
  , visited_(0)
{
  if( runningIndex_==std::numeric_limits<IdType>::max() )
    std::cerr<<"Attention::View::index_ is about to overflow! (Next: "<<runningIndex_<<", and the one after: "<<runningIndex_+1<<".";
}

View::View( IdType id )
  : index_(id)
  , is_reachable_(true)
  , is_bad_(false)
  , non_viewspace_(false)
  , visited_(0)
{
  
}

movements::Pose& View::pose()
{
  return pose_;
}

const movements::Pose& View::pose() const
{
  return pose_;
}

std::string& View::sourceFrame()
{
  return source_frame_;
}

const std::string& View::sourceFrame() const
{
  return source_frame_;
}

bool& View::reachable()
{
  return is_reachable_;
}

const bool& View::reachable() const
{
  return is_reachable_;
}

unsigned int& View::timesVisited()
{
  return visited_;
}

const unsigned int& View::timesVisited() const
{
  return visited_;
}

bool& View::bad()
{
  return is_bad_;
}

const bool& View::bad() const
{
  return is_bad_;
}

bool& View::nonViewSpace()
{
  return non_viewspace_;
}

const bool& View::nonViewSpace() const
{
  return non_viewspace_;
}

View::IdType View::index() const
{
  return index_;
}

std::vector<std::string>& View::additionalFieldsNames()
{
  return additional_fields_names_;
}

const std::vector<std::string>& View::additionalFieldsNames() const
{
  return additional_fields_names_;
}

std::vector<double>& View::additionalFieldsValues()
{
  return additional_fields_values_;
}

const std::vector<double>& View::additionalFieldsValues() const
{
  return additional_fields_values_;
}

boost::shared_ptr<ig_active_reconstruction::views::View::ViewInfo>& View::associatedData()
{
  return associated_data_;
}

}

}

std::ostream& operator<<(std::ostream& _out, ig_active_reconstruction::views::View& view )
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