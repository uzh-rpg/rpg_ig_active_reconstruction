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

#pragma once

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "movements/relative_movement.h"

namespace movements
{
  class CombinedRelativeMovement; // forward declaration
  
  /** class to represent a geometry pose in 3d space */
  class Pose
  {
  public:
    Pose();
    Pose( Eigen::Vector3d _position, Eigen::Quaterniond _orientation );
    
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    
    bool operator!=( const movements::Pose& _to_compare );
    bool operator==( const movements::Pose& _to_compare );
    
    /** executes a relative movement on the pose */
    Pose operator+( movements::RelativeMovement  _second );
    /** executes a combined relative movement on the pose */
    Pose operator+( movements::CombinedRelativeMovement  _second );
    
    /** executes a relative movement on the pose */
    Pose& operator+=( movements::RelativeMovement  _second );
    /** executes a relative movement on the pose */
    Pose& operator+=( movements::CombinedRelativeMovement  _second );
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  
  typedef std::vector<movements::Pose,Eigen::aligned_allocator<movements::Pose> > PoseVector;
}

std::ostream& operator<<(std::ostream& _out, movements::Pose& _pose );