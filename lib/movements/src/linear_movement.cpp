/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
* 
linear_movement is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
linear_movement is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with linear_movement. If not, see <http://www.gnu.org/licenses/>.
*/

#include "movements/linear_movement.h"

namespace movements
{

LinearMovement::LinearMovement()
{
  
}

LinearMovement::LinearMovement( double _x, double _y, double _z, double _velocity ):
direction_(_x,_y,_z),
velocity_(_velocity)
{
  
}

LinearMovement::LinearMovement( Eigen::Vector3d _direction, double _velocity ):
direction_(_direction),
velocity_(_velocity)
{
  
}

LinearMovement::LinearMovement( Translation _direction, double _speed ):
direction_( _direction.x(), _direction.y(), _direction.z() ),
velocity_(_speed)
{
  
}

double& LinearMovement::x()
{
  return direction_(0);
}

double& LinearMovement::y()
{
  return direction_(1);
}

double& LinearMovement::z()
{
  return direction_(2);
}

double& LinearMovement::velocity()
{
  return velocity_;
}

std::string LinearMovement::type()
{
  return "movements::LinearMovement";
}

RelativeMovement LinearMovement::operator()( double _time )
{
  normalizeDirection(); // could be sped up...
  double distance_covered = _time*velocity_;
  
  return Translation::create( distance_covered*direction_ );
}

KinematicMovementDescription LinearMovement::create( double _x, double _y, double _z, double _velocity )
{
  return KinematicMovementDescription( new LinearMovement(_x,_y,_z,_velocity) );
}

KinematicMovementDescription LinearMovement::create( Eigen::Vector3d _direction, double _velocity )
{
  return KinematicMovementDescription( new LinearMovement(_direction,_velocity) );
}

KinematicMovementDescription LinearMovement::create( Translation _direction, double _speed )
{
  return KinematicMovementDescription( new LinearMovement(_direction,_speed) );
}

void LinearMovement::normalizeDirection()
{
  direction_.normalize();
}


}