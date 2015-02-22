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

#pragma once

#include "utils/relative_movement.h"

namespace st_is
{
/** to describe a kinematic, time based movement */
class KinematicMovementDescription
{
public:
  /** returns the relative movement at time _time */
  RelativeMovement operator()( double _time )=0;
};

}