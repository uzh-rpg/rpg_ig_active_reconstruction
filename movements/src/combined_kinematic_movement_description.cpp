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
#include <stdexcept>

#include "movements/geometry_pose.h"
#include "movements/combined_kinematic_movement_description.h"
#include "movements/kinematic_movement_description.h"
#include "movements/combined_relative_movement.h"
#include <boost/foreach.hpp>

namespace movements
{

CombinedKinematicMovementDescription::CombinedKinematicMovementDescription()
{
  
}

CombinedRelativeMovement CombinedKinematicMovementDescription::operator()( double _time )
{
  CombinedRelativeMovement comb_rel_movement;
  
  auto rel_move_it = relative_movement_queue_.begin();
  auto kin_move_it = kinematic_movement_queue_.begin();
  
  auto rel_end_it = relative_movement_queue_.end();
  auto kin_end_it = kinematic_movement_queue_.end();
  
  while( rel_move_it!=rel_end_it && kin_move_it!=kin_end_it )
  {
    if( rel_move_it==rel_end_it )
    {
      comb_rel_movement += (*kin_move_it).second(_time);
    }
    else if( kin_move_it==kin_end_it )
    {
      comb_rel_movement += (*rel_move_it).second;
    }
    else if( (*rel_move_it).first < (*kin_move_it).first )
    {
      comb_rel_movement += (*rel_move_it).second;
      rel_move_it++;
    }
    else
    {
      comb_rel_movement += (*kin_move_it).second(_time);
      kin_move_it++;
    }
  }
  return comb_rel_movement;
}

std::vector<CombinedRelativeMovement> CombinedKinematicMovementDescription::relativePath( double _start_time, double _end_time, double _step_size )
{
  std::vector<CombinedRelativeMovement> relative_movement_queue;
  for( double t=_start_time; t<=_end_time; t+=_step_size )
  {
    relative_movement_queue.push_back( (*this)(t) );
  }
  return relative_movement_queue;
}

movements::PoseVector CombinedKinematicMovementDescription::path( movements::Pose _base_pose, double _start_time, double _end_time, double _step_size )
{
  movements::PoseVector cartesian_path;
  for( double t=_start_time; t<=_end_time; t+=_step_size )
  {
    movements::CombinedRelativeMovement move = (*this)(t);
    cartesian_path.push_back( _base_pose + move );
  }
  return cartesian_path;
}


CombinedKinematicMovementDescription& CombinedKinematicMovementDescription::operator=( CombinedRelativeMovement const& _to_equal )
{
  relative_movement_queue_.clear();
  kinematic_movement_queue_.clear();
  
  BOOST_FOREACH( auto rel_movement, _to_equal.relative_movement_queue_ )
  {
    relative_movement_queue_.push_back( std::pair<int,RelativeMovement>(relative_movement_queue_.size(),rel_movement) );
  }
  
  return *this;
}

CombinedKinematicMovementDescription& CombinedKinematicMovementDescription::operator=( RelativeMovement const& _to_equal )
{
  relative_movement_queue_.clear();
  kinematic_movement_queue_.clear();
  
  relative_movement_queue_.push_back( std::pair<int,RelativeMovement>(0,_to_equal) );
  
  return *this;
}

CombinedKinematicMovementDescription& CombinedKinematicMovementDescription::operator=( KinematicMovementDescription const& _to_equal )
{
  relative_movement_queue_.clear();
  kinematic_movement_queue_.clear();
  
  kinematic_movement_queue_.push_back( std::pair<int,KinematicMovementDescription>(0,_to_equal) );
  
  return *this;
}

CombinedKinematicMovementDescription CombinedKinematicMovementDescription::operator+( CombinedRelativeMovement const& _to_add )
{
  CombinedKinematicMovementDescription new_chain(*this);  
  new_chain+=_to_add;  
  return new_chain;
}

CombinedKinematicMovementDescription CombinedKinematicMovementDescription::operator+( RelativeMovement const& _to_add )
{
  CombinedKinematicMovementDescription new_chain(*this);  
  new_chain+=_to_add;  
  return new_chain;
}

CombinedKinematicMovementDescription CombinedKinematicMovementDescription::operator+( KinematicMovementDescription const& _to_add )
{
  CombinedKinematicMovementDescription new_chain(*this);  
  new_chain+=_to_add;  
  return new_chain;
}

CombinedKinematicMovementDescription CombinedKinematicMovementDescription::operator+( CombinedKinematicMovementDescription const& _to_add )
{
  CombinedKinematicMovementDescription new_chain(*this);  
  new_chain+=_to_add;  
  return new_chain;
}

CombinedKinematicMovementDescription& CombinedKinematicMovementDescription::operator+=( CombinedRelativeMovement const& _to_add )
{
  BOOST_FOREACH( auto rel_movement, _to_add.relative_movement_queue_ )
  {
    relative_movement_queue_.push_back( std::pair<int,RelativeMovement>(nrOfMovements(),rel_movement) );
  }
  return *this;
}

CombinedKinematicMovementDescription& CombinedKinematicMovementDescription::operator+=( RelativeMovement const& _to_add )
{
  relative_movement_queue_.push_back( std::pair<int,RelativeMovement>(nrOfMovements(),_to_add) );
  return *this;
}

CombinedKinematicMovementDescription& CombinedKinematicMovementDescription::operator+=( KinematicMovementDescription const& _to_add )
{
  kinematic_movement_queue_.push_back( std::pair<int,KinematicMovementDescription>(nrOfMovements(),_to_add) );
  return *this;
}

CombinedKinematicMovementDescription& CombinedKinematicMovementDescription::operator+=( CombinedKinematicMovementDescription const& _to_add )
{
  unsigned int old_size = nrOfMovements();
  
  BOOST_FOREACH( auto rel_movement, _to_add.relative_movement_queue_ )
  {
    relative_movement_queue_.push_back( std::pair<int,RelativeMovement>( old_size+rel_movement.first, rel_movement.second ) );
  }
  BOOST_FOREACH( auto kin_movement, _to_add.kinematic_movement_queue_ )
  {
    kinematic_movement_queue_.push_back( std::pair<int,KinematicMovementDescription>( old_size+kin_movement.first, kin_movement.second ) );
  }
  return *this;
}

unsigned int CombinedKinematicMovementDescription::nrOfMovements()
{
  return relative_movement_queue_.size() + kinematic_movement_queue_.size();
}



}