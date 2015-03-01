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

#include "movements/translation.h"
#include "movements/geometry_pose.h"

namespace movements
{

Translation::Translation()
{
  
}

Translation::Translation( double _x, double _y, double _z ):
translation_(_x,_y,_z)
{
  
}

Translation::Translation( Eigen::Vector3d _translation ):
translation_(_translation)
{
  
}

double& Translation::x()
{
  return translation_(0);
}

double& Translation::y()
{
  return translation_(1);
}

double& Translation::z()
{
  return translation_(2);
}



std::string Translation::type()
{
  return "movements::Translation";
}

Pose Translation::applyToBasePose( Pose const& _base )
{
  Pose copy = _base;
  copy.position += translation_;
  return copy;
}

RelativeMovement Translation::create( double _x, double _y, double _z )
{
  return RelativeMovement( new Translation(_x,_y,_z) );
}

RelativeMovement Translation::create( Eigen::Vector3d _translation )
{
  return RelativeMovement( new Translation(_translation) );
}


}