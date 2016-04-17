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

#include "movements/geometry_pose.h"
#include "movements/combined_relative_movement.h"
#include "movements/combined_kinematic_movement_description.h"

namespace movements
{

CombinedRelativeMovement::CombinedRelativeMovement()
{
}

movements::Pose CombinedRelativeMovement::applyToBasePose( movements::Pose& _base )
{
  movements::Pose end_pose = _base;
  for( auto& rel_movement: relative_movement_queue_ )
  {
    end_pose+=rel_movement;
  }
  return end_pose;
}

CombinedRelativeMovement& CombinedRelativeMovement::operator=( RelativeMovement const& _to_equal )
{
  relative_movement_queue_.clear();
  relative_movement_queue_.push_back( _to_equal );
}

CombinedRelativeMovement CombinedRelativeMovement::operator+( CombinedRelativeMovement const& _to_add )
{
  CombinedRelativeMovement rel_movement = *this;
  rel_movement+=_to_add;
  return rel_movement;
}

CombinedRelativeMovement CombinedRelativeMovement::operator+( RelativeMovement const& _to_add )
{
  CombinedRelativeMovement rel_movement = *this;
  rel_movement+=_to_add;
  return rel_movement;
}

CombinedKinematicMovementDescription CombinedRelativeMovement::operator+( KinematicMovementDescription const& _to_add )
{
  CombinedKinematicMovementDescription kin_movement_chain;
  kin_movement_chain = (*this);
  kin_movement_chain+=_to_add;
  return kin_movement_chain;
}

CombinedKinematicMovementDescription CombinedRelativeMovement::operator+( CombinedKinematicMovementDescription const& _to_add )
{
  CombinedKinematicMovementDescription kin_movement_chain;
  kin_movement_chain = *this;
  kin_movement_chain+=_to_add;
  return kin_movement_chain;
}

CombinedRelativeMovement& CombinedRelativeMovement::operator+=( CombinedRelativeMovement const& _to_add )
{
  for( auto& rel_movement: _to_add.relative_movement_queue_ )
  {
    relative_movement_queue_.push_back( rel_movement );
  }
  return *this;
}

CombinedRelativeMovement& CombinedRelativeMovement::operator+=( RelativeMovement const& _to_add )
{
  relative_movement_queue_.push_back(_to_add);
  return *this;
}

}