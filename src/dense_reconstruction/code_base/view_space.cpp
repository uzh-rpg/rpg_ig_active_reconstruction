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
#include <fstream>
#include "ros/ros.h"

namespace dense_reconstruction
{

ViewSpace::ViewSpace()
{
  
}

void ViewSpace::fromMsg( const ViewSpaceMsg& _msg )
{
  BOOST_FOREACH( auto view_msg, _msg.views )
  {
    view_space_.push_back( View(view_msg) );
  }
}

ViewSpaceMsg ViewSpace::toMsg()
{
  ViewSpaceMsg msg;
  BOOST_FOREACH( auto view, view_space_ )
  {
    msg.views.push_back( view.toMsg() );
  }
  return msg;
}

std::vector<View, Eigen::aligned_allocator<View> > ViewSpace::getViewSpace()
{
  return view_space_;
}

void ViewSpace::getGoodViewSpace( std::vector<unsigned int>& _out, bool _ignore_visited )
{
  for( unsigned int i=0; i<view_space_.size(); ++i )
  {
    if( view_space_[i].reachable() && (view_space_[i].timesVisited()==0||!_ignore_visited) && !view_space_[i].bad() )
    {
      _out.push_back(i);
    }
  }
}

View ViewSpace::getView( unsigned int _index )
{
  if( _index>=view_space_.size() )
  {
    throw std::invalid_argument("ViewSpace::getView: the given index is out of range");
  }
  return view_space_[_index];
}

unsigned int ViewSpace::timesVisited( unsigned int _index )
{
  if( _index<view_space_.size() )
  {
    return view_space_[_index].timesVisited();
  }
  return 0;
}

void ViewSpace::setBad( unsigned int _index )
{
  if( _index<view_space_.size() )
  {
    view_space_[_index].bad()=true;
  }
}

void ViewSpace::setGood( unsigned int _index )
{
  if( _index<view_space_.size() )
  {
    view_space_[_index].bad()=false;
  }
}

void ViewSpace::setVisited( unsigned int _index )
{
  if( _index<view_space_.size() )
  {
    int count = view_space_[_index].timesVisited()+1;
    view_space_[_index].timesVisited() = count;
  }
}

void ViewSpace::setUnReachable( unsigned int _index )
{
  if( _index<view_space_.size() )
  {
    view_space_[_index].reachable()=false;
  }
}

void ViewSpace::setReachable( unsigned int _index )
{
  if( _index<view_space_.size() )
  {
    view_space_[_index].reachable()=true;
  }
}


void ViewSpace::push_back( View _new_vp )
{
  _new_vp.index = view_space_.size();
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
    
    new_pose.index = i;
    new_pose.bad() = false;
    new_pose.reachable() = true;
    new_pose.timesVisited() = 0;
    view_space_.push_back(new_pose);
  }
}

}