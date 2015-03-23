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

#include "dense_reconstruction/view_space.h"
#include "boost/foreach.hpp"

namespace dense_reconstruction
{

ViewSpace::ViewSpace()
{
  
}

void ViewSpace::fromMsg( const ViewSpaceMsgConstPtr& _msg )
{
  BOOST_FOREACH( auto view_msg, _msg->views )
  {
    view_space_.push_back( View(view_msg) );
  }
}

std::vector<View, Eigen::aligned_allocator<View> > ViewSpace::getViewSpace()
{
  return view_space_;
}

void ViewSpace::push_back( View _new_vp )
{
  view_space_.push_back(_new_vp);
}

View ViewSpace::getAClosestNeighbour( View& _view )
{
  if( view_space_.empty() )
    throw std::runtime_error("ViewSpace::getAClosestNeighbour::Cannot find a closest neighbour since the view space is empty.");
    
  Eigen::Vector3d probe = _view.pose().position;
  View closest = view_space_[0];
  Eigen::Vector3d dist_vec = probe - closest.pose().position;
  double distance = dist_vec.norm();
  
  BOOST_FOREACH( auto view, view_space_ )
  {
    dist_vec = probe - view.pose().position;
    double norm = dist_vec.norm();
    if( norm<distance )
    {
      distance=norm;
      closest = view;
    }
  }
  return closest;
}

unsigned int ViewSpace::size()
{
  return view_space_.size();
}

void ViewSpace::getViewsInRange( View& _reference_view, double _distance, std::vector<View, Eigen::aligned_allocator<View> >& _sub_space )
{
  BOOST_FOREACH( auto view, view_space_ )
  {
    Eigen::Vector3d dist_vec = view.pose().position - _reference_view.pose().position;
    if( dist_vec.norm()<=_distance )
    {
      _sub_space.push_back(view);
    }
  }
}

}