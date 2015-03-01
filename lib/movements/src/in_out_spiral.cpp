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

InOutSpiral::InOutSpiral( Eigen::Quaterniond _orientation, double _max_radius, double _angle_speed, double _radial_speed ):
orientation_(_orientation),
max_radius_(_max_radius),
angle_speed_(_angle_speed),
radial_speed_(_radial_speed)
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
  
  double x = current_radius * cos(current_angle);
  double y = current_radius * sin(current_angle);
  
  Eigen::Vector3d relative_movement_spiral_coord( x,y,0 );
  Eigen::Vector3d relative_movement_parent_coord = orientation_*relative_movement_spiral_coord;
  
  return Translation::create( relative_movement_parent_coord );
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