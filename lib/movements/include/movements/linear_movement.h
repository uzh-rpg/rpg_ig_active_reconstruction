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

#pragma once

#include "movements/kinematic_movement_description.h"
#include "movements/translation.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace movements
{

/// represents a kinematic description of a relative linear movement
class LinearMovement:public KinematicMovementDescription::KinematicMovementDescriptionInstance
{
public:
  LinearMovement();
  /** constructor: lengths in [m], velocity in [m/s] */
  LinearMovement( double _x, double _y, double _z, double _velocity );
  /** constructor: lengths in [m], velocity in [m/s] */
  LinearMovement( Eigen::Vector3d _direction, double _velocity );
  /** constructor: lengths in [m], velocity in [m/s] */
  LinearMovement( Translation _direction, double _speed );
  
  /** access to the x-translation element [m]*/
  double& x();
  /** access to the y-translation element  [m]*/
  double& y();
  /** access to the z-translation element  [m]*/
  double& z();
  /** access to the speed  [m/s]*/
  double& velocity();
  
  /// returns the type of the enclosed kinematic movement
  virtual std::string type();
  
  /** returns the relative movement at time _time */
  virtual RelativeMovement operator()( double _time );
  
  /** directly returns a kinematic movement description containing a linear movement */
  static KinematicMovementDescription create( double _x, double _y, double _z, double _velocity );
  /** directly returns a kinematic movement description containing a linear movement */
  static KinematicMovementDescription create( Eigen::Vector3d _direction, double _velocity );
  /** directly returns a kinematic movement description containing a linear movement */
  static KinematicMovementDescription create( Translation _direction, double _speed );
private:
  Eigen::Vector3d direction_; /// [m]
  double velocity_; /// [m/s]
  
  void normalizeDirection();
};

}