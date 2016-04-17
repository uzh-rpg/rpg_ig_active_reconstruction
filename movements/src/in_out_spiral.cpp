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

#include "movements/in_out_spiral.h"
#include <cmath>

namespace movements
{

InOutSpiral::InOutSpiral( Eigen::Quaterniond _orientation, double _max_radius, double _angle_speed, double _radial_speed, Plane _plane ):
orientation_(_orientation),
max_radius_(_max_radius),
angle_speed_(_angle_speed),
radial_speed_(_radial_speed),
plane_to_use_(_plane)
{
  half_time_ = max_radius_ / radial_speed_;
}

std::string InOutSpiral::type()
{
  return "movements::InOutSpiral";
}

RelativeMovement InOutSpiral::operator()( double _time )
{
  double current_radius = getRadius(_time);
  double current_angle = _time*angle_speed_;
  
  double first_axis = current_radius * cos(current_angle);
  double second_axis = current_radius * sin(current_angle);
  double normal_axis = 0;
  
  double x,y,z;
  
  switch(plane_to_use_)
  {
    case XYPlane:
      x=first_axis;
      y=second_axis;
      z=normal_axis;
      break;
    case YZPlane:
      x=normal_axis;
      y=first_axis;
      z=second_axis;
      break;
    case ZXPlane:
      x=second_axis;
      y=normal_axis;
      z=first_axis;
      break;
    case YXPlane:
      x=second_axis;
      y=first_axis;
      z=normal_axis;
      break;
    case ZYPlane:
      x=normal_axis;
      y=second_axis;
      z=first_axis;
      break;
    case XZPlane:
      x=first_axis;
      y=normal_axis;
      z=second_axis;
      break;
  }
  
  Eigen::Vector3d relative_movement_spiral_coord( x,y,z );
  Eigen::Vector3d relative_movement_parent_coord = orientation_*relative_movement_spiral_coord;
  
  return Translation::create( relative_movement_parent_coord );
}

KinematicMovementDescription InOutSpiral::create(  Eigen::Quaterniond _orientation, double _max_radius, double _angle_speed, double _radial_speed, Plane _plane )
{
  return KinematicMovementDescription( new InOutSpiral(_orientation,_max_radius,_angle_speed,_radial_speed,_plane) );
}

double InOutSpiral::getRadius( double _time )
{
  double relative_time = _time/half_time_;
  bool is_growing = ( fmod(relative_time,2)<1 )?true:false;
  
  int half_iteration_nr = (int)relative_time;
  
  if( is_growing )
  {
    return radial_speed_*(_time-half_iteration_nr*half_time_);
  }
  else
  {
    return max_radius_ - radial_speed_*(_time-half_iteration_nr*half_time_);
  }
}

}