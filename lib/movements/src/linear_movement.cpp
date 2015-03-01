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

#include "movements/linear_movement.h"

namespace movements
{

Linear::Linear()
{
  
}

Linear::Linear( double _x, double _y, double _z, double _velocity ):
direction_(_x,_y,_z),
velocity_(_velocity)
{
  
}

Linear::Linear( Eigen::Vector3d _direction, double _velocity ):
direction_(_direction),
velocity_(_velocity)
{
  
}

Linear::Linear( Translation _direction, double _speed ):
direction_( _direction.x(), _direction.y(), _direction.z() ),
velocity_(_speed)
{
  
}

double& Linear::x()
{
  return direction_(0);
}

double& Linear::y()
{
  return direction_(1);
}

double& Linear::z()
{
  return direction_(2);
}

double& Linear::velocity()
{
  return velocity_;
}

std::string Linear::type()
{
  return "movements::Linear";
}

RelativeMovement Linear::operator()( double _time )
{
  normalizeDirection(); // could be sped up...
  double distance_covered = _time*velocity_;
  
  return Translation::create( distance_covered*direction_ );
}

KinematicMovementDescription Linear::create( double _x, double _y, double _z, double _velocity )
{
  return KinematicMovementDescription( new Linear(_x,_y,_z,_velocity) );
}

KinematicMovementDescription Linear::create( Eigen::Vector3d _direction, double _velocity )
{
  return KinematicMovementDescription( new Linear(_direction,_velocity) );
}

KinematicMovementDescription Linear::create( Translation _direction, double _speed )
{
  return KinematicMovementDescription( new Linear(_direction,_speed) );
}

void Linear::normalizeDirection()
{
  direction_.normalize();
}


}