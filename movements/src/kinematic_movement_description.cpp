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
#include "movements/kinematic_movement_description.h"
#include "movements/combined_kinematic_movement_description.h"
#include "movements/combined_relative_movement.h"

namespace movements
{
 
KinematicMovementDescription::KinematicMovementDescription( KinematicMovementDescriptionInstance* _to_enwrap )
{
  enwrapped_kinematic_movement_description_ = boost::shared_ptr<KinematicMovementDescriptionInstance>(_to_enwrap);
}

std::string KinematicMovementDescription::type()
{
  return enwrapped_kinematic_movement_description_->type();
}

RelativeMovement KinematicMovementDescription::operator()( double _time )
{
  return (*enwrapped_kinematic_movement_description_)(_time);
}

boost::shared_ptr<KinematicMovementDescription::KinematicMovementDescriptionInstance> KinematicMovementDescription::operator->()
{
  return enwrapped_kinematic_movement_description_;
}

KinematicMovementDescription::KinematicMovementDescriptionInstance& KinematicMovementDescription::operator*()
{
  return *enwrapped_kinematic_movement_description_;
}

std::vector<RelativeMovement> KinematicMovementDescription::relativePath( double _start_time, double _end_time, double _step_size )
{
  if( _step_size<=0 )
  {
    throw std::invalid_argument("KinematicMovementDescription::relativePath::Called with invalid argument: _step_size is less or equal to zero.");
  }
  if( _start_time>_end_time )
  {
    throw std::invalid_argument("KinematicMovementDescription::relativePath::Called with invalid arguments: _start_time is larger than _end_time.");
  }
  return enwrapped_kinematic_movement_description_->relativePath(_start_time,_end_time,_step_size);
}

movements::PoseVector KinematicMovementDescription::path( movements::Pose _base_pose, double _start_time, double _end_time, double _step_size )
{
  if( _step_size<=0 )
  {
    throw std::invalid_argument("KinematicMovementDescription::path::Called with invalid argument: _step_size is less or equal to zero.");
  }
  if( _start_time>_end_time )
  {
    throw std::invalid_argument("KinematicMovementDescription::path::Called with invalid arguments: _start_time is larger than _end_time.");
  }
  return enwrapped_kinematic_movement_description_->path(_base_pose,_start_time,_end_time,_step_size);
}

std::vector<RelativeMovement> KinematicMovementDescription::KinematicMovementDescriptionInstance::relativePath( double _start_time, double _end_time, double _step_size )
{
  std::vector<RelativeMovement> relative_path;
  for( double t=_start_time; t<=_end_time; t+=_step_size )
  {
    relative_path.push_back( (*this)(t) );
  }
  return relative_path;
}

movements::PoseVector KinematicMovementDescription::KinematicMovementDescriptionInstance::path( movements::Pose _base_pose, double _start_time, double _end_time, double _step_size )
{
  movements::PoseVector cartesian_path;
  for( double t=_start_time; t<=_end_time; t+=_step_size )
  {
    RelativeMovement move = (*this)(t);
    cartesian_path.push_back( _base_pose+move );
  }
  return cartesian_path;
}
  
}