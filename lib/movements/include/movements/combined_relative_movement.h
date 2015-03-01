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

#include "movements/relative_movement.h"
#include "movements/kinematic_movement_description.h"
#include <deque>

namespace movements
{
class CombinedKinematicMovementDescription;

/// Class to hold a series of fixed relative movements
class CombinedRelativeMovement
{
public:
  friend CombinedKinematicMovementDescription;
  
  CombinedRelativeMovement();
  
  /** applies the relative movement queue to a base pose */
  movements::Pose applyToBasePose( movements::Pose& _base );
  
  /** replaces the current relative movement chain represented by the object with _to_equal as the one, single chain element */
  CombinedRelativeMovement& operator=( RelativeMovement const& _to_equal );
  
  /** creates a new combined relative movement with the same relative movement chain as the current object but with _to_add appended */
  CombinedRelativeMovement operator+( CombinedRelativeMovement const& _to_add );
  /** creates a new combined relative movement with the same relative movement chain as the current object but with _to_add appended */
  CombinedRelativeMovement operator+( RelativeMovement const& _to_add );
  /** creates a new combined relative movement with the same relative movement chain as the current object but with _to_add appended */
  CombinedKinematicMovementDescription operator+( KinematicMovementDescription const& _to_add );
  /** creates a new combined relative movement with the same relative movement chain as the current object but with _to_add appended */
  CombinedKinematicMovementDescription operator+( CombinedKinematicMovementDescription const& _to_add );
  
  /** appends _to_add to the internal relative movement chain */
  CombinedRelativeMovement& operator+=( CombinedRelativeMovement const& _to_add );
  /** appends _to_add to the internal relative movement chain */
  CombinedRelativeMovement& operator+=( RelativeMovement const& _to_add );
private:
  std::deque< RelativeMovement > relative_movement_queue_;
};

typedef CombinedRelativeMovement CombRelMove;
  
}