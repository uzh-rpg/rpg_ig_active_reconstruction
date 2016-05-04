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

#include "ig_active_reconstruction/view_space.hpp"
#include <fstream>
#include <stdexcept>

namespace ig_active_reconstruction
{
  
namespace views
{

ViewSpace::ViewSpace()
{
  
}

std::vector<View, Eigen::aligned_allocator<View> > ViewSpace::getViewSpace()
{
  return view_space_;
}

void ViewSpace::getGoodViewSpace( IdSet& out, bool ignore_visited )
{
  for( View& view: view_space_ )
  {
    if( view.reachable() && (view.timesVisited()==0||!ignore_visited) && !view.bad() )
    {
      out.push_back(view.index());
    }
  }
}

View ViewSpace::getView( View::IdType index )
{
  try
  {
    return *(views_index_map_.at(index));
  }
  catch(...)
  {
    throw std::out_of_range("ViewSpace::getView: the given index is out of range");
  }
}

bool ViewSpace::deleteView( View::IdType index )
{
  decltype(view_space_)::iterator it = view_space_.begin();
  decltype(view_space_)::iterator end = view_space_.end();
  
  for( ; it!=end; ++it )
  {
    if( it->index()==index )
    {
      view_space_.erase(it);
      recalculateIndexMap();
      return true;
    }
  }
  return false;
}

bool ViewSpace::deleteViews( std::vector<View::IdType>& index_set )
{
  bool found = true;
  for( View::IdType& id: index_set )
  {
    found = found && deleteView(id);
  }
  
  return found;
}

unsigned int ViewSpace::timesVisited( View::IdType index )
{
  try
  {
    return views_index_map_.at(index)->timesVisited();
  }
  catch(...)
  {
    return 0;
  }
}

void ViewSpace::setBad( View::IdType index )
{
  try
  {
    views_index_map_.at(index)->bad()=true;
  }
  catch(...)
  {
    return;
  }
}

void ViewSpace::setGood( View::IdType index )
{
  try
  {
    views_index_map_.at(index)->bad()=false;
  }
  catch(...)
  {
    return;
  }
}

void ViewSpace::setVisited( View::IdType index )
{
  try
  {
    views_index_map_.at(index)->timesVisited() += 1;
  }
  catch(...)
  {
    return;
  }
}

void ViewSpace::setUnReachable( View::IdType index )
{
  try
  {
    views_index_map_.at(index)->reachable() = false;
  }
  catch(...)
  {
    return;
  }
}

void ViewSpace::setReachable( View::IdType index )
{
  try
  {
    views_index_map_.at(index)->reachable() = true;
  }
  catch(...)
  {
    return;
  }
}


void ViewSpace::push_back( View new_vp )
{
  view_space_.push_back(new_vp);
  views_index_map_[new_vp.index()] = &view_space_.back();
}

View ViewSpace::getAClosestNeighbour( View& _view )
{
  if( view_space_.empty() )
    throw std::runtime_error("ViewSpace::getAClosestNeighbour::Cannot find a closest neighbour since the view space is empty.");
    
  Eigen::Vector3d probe = _view.pose().position;
  View closest = view_space_[0];
  Eigen::Vector3d dist_vec = probe - closest.pose().position;
  double distance = dist_vec.norm();
  
  for( auto& view: view_space_ )
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
  for( auto& view: view_space_ )
  {
    Eigen::Vector3d dist_vec = view.pose().position - _reference_view.pose().position;
    if( dist_vec.norm()<=_distance )
    {
      _sub_space.push_back(view);
    }
  }
}

void ViewSpace::saveToFile( std::string _filename )
{
  std::ofstream out( _filename, std::ofstream::trunc );
  
  out<<view_space_.size();
  
  for( unsigned int i=0; i<view_space_.size(); ++i )
  {
    out<<"\n";
    movements::Pose pose = view_space_[i].pose();
    out << pose.position.x();
    out << " " << pose.position.y();
    out << " " << pose.position.z();
    out << " " << pose.orientation.x();
    out << " " << pose.orientation.y();
    out << " " << pose.orientation.z();
    out << " " << pose.orientation.w();
  }
    
  out.close();
}

void ViewSpace::loadFromFile( std::string _filename )
{
  std::ifstream in(_filename, std::ifstream::in);
  
  unsigned int nr_of_views;
  bool success = (in >> nr_of_views);
  
  if(!success)
    return;
  
  for( unsigned int i=0; i<nr_of_views; ++i )
  {
    View new_pose;
    success = success && ( in>>new_pose.pose().position.x() );
    success = success && ( in>>new_pose.pose().position.y() );
    success = success && ( in>>new_pose.pose().position.z() );
    success = success && ( in>>new_pose.pose().orientation.x() );
    success = success && ( in>>new_pose.pose().orientation.y() );
    success = success && ( in>>new_pose.pose().orientation.z() );
    success = success && ( in>>new_pose.pose().orientation.w() );
    
    if(!success)
      return;
    
    new_pose.bad() = false;
    new_pose.reachable() = true;
    new_pose.timesVisited() = 0;
    
    view_space_.push_back(new_pose);
    views_index_map_[new_pose.index()] = &view_space_.back();
  }
}

ViewSpace::Iterator ViewSpace::begin()
{
  return view_space_.begin();
}

ViewSpace::Iterator ViewSpace::end()
{
  return view_space_.end();
}

bool ViewSpace::empty() const
{
  return view_space_.empty();
}

void ViewSpace::recalculateIndexMap()
{
  views_index_map_.clear();
  
  for( View& view: view_space_ )
  {
    views_index_map_[view.index()] = &view;
  }
}

}

}