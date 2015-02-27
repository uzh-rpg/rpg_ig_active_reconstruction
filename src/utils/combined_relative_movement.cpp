/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
* 
combined_relative_movement is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
combined_relative_movement is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with combined_relative_movement. If not, see <http://www.gnu.org/licenses/>.
*/

#include "utils/combined_relative_movement.h"
#include "utils/combined_kinematic_movement_description.h"
#include <boost/foreach.hpp>

namespace st_is
{

CombinedRelativeMovement::CombinedRelativeMovement()
{
}

st_is::GeometryPose CombinedRelativeMovement::applyToBasePose( st_is::GeometryPose& _base )
{
  st_is::GeometryPose end_pose = _base;
  BOOST_FOREACH( auto rel_movement, relative_movement_queue_ )
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
  CombinedKinematicMovementDescription kin_movement_chain = *this;
  kin_movement_chain+=_to_add;
  return kin_movement_chain;
}

CombinedKinematicMovementDescription CombinedRelativeMovement::operator+( CombinedKinematicMovementDescription const& _to_add )
{
  CombinedKinematicMovementDescription kin_movement_chain = *this;
  kin_movement_chain+=_to_add;
  return kin_movement_chain;
}

CombinedRelativeMovement& CombinedRelativeMovement::operator+=( CombinedRelativeMovement const& _to_add )
{
  BOOST_FOREACH( auto rel_movement, _to_add.relative_movement_queue_ )
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