/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
* 
kinematic_movement_description is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
kinematic_movement_description is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with kinematic_movement_description. If not, see <http://www.gnu.org/licenses/>.
*/

#include "utils/kinematic_movement_description.h"
#include "utils/combined_kinematic_movement_description.h"
#include "utils/combined_relative_movement.h"

namespace st_is
{
 
KinematicMovementDescription::KinematicMovementDescription( KinematicMovementDescription* _to_enwrap )
{
  enwrapped_kinematic_movement_description_ = boost::shared_ptr<KinematicMovementDescriptionInstance>(_to_enwrap);
}

std::string KinematicMovementDescription::type()
{
  return enwrapped_kinematic_movement_description_->type();
}

std::vector<RelativeMovement> relativePath( double _start_time, double _end_time, double _step_size )
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

std::vector<st_is::GeometryPose> KinematicMovementDescription::path( st_is::GeometryPose _base_pose, double _start_time, double _end_time, double _step_size )
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

std::vector<RelativeMovement> relativePath( double _start_time, double _end_time, double _step_size )
{
  std::vector<RelativeMovement> relative_path;
  for( double t=_start_time; t<=_end_time; t+=_step_size )
  {
    relative_path.push_back( this->(t) );
  }
  return relative_path;
}

std::vector<st_is::GeometryPose> KinematicMovementDescription::KinematicMovementDescriptionInstance::path( st_is::GeometryPose _base_pose, double _start_time, double _end_time, double _step_size )
{
  std::vector<st_is::GeometryPose> cartesian_path;
  for( double t=_start_time; t<=_end_time; t+=_step_size )
  {
    cartesian_path.push_back( _base_pose + this->(t) );
  }
  return cartesian_path;
}
  
}