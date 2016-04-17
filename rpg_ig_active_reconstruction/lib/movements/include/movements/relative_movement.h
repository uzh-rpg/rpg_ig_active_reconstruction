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

#include <boost/shared_ptr.hpp>

namespace movements
{

class Pose;
class CombinedKinematicMovementDescription;
class CombinedRelativeMovement;
class KinematicMovementDescription;
  
  
/** A class that serves as package wrapper for RelativeMovementInstances that naturally need to be moved around as pointers. The use of this class keeps the whole pointer arithmetic internal and thus easens the use.
 */
class RelativeMovement
{
public:
  class RelativeMovementInstance;
  
  /// constructor
  RelativeMovement( RelativeMovementInstance* _to_enwrap );
  
  /// returns the type of the enclosed movement
  std::string type();
  
  
  /** applies the relative movement to a base pose */
  movements::Pose applyToBasePose( movements::Pose& _base );
  
  /** returns a pointer to the internally hold RelativeMovementInstance */
  boost::shared_ptr<RelativeMovementInstance> operator*();
  
  /** creates a relative movement event chain where the movement represented by the class object is prepended to the argument _to_add */
  CombinedRelativeMovement operator+( RelativeMovement const& _to_add );
  
  /** creates a relative movement event chain where the movement represented by the class object is prepended to the argument _to_add */
  CombinedRelativeMovement operator+( CombinedRelativeMovement const& _to_add );
  
  /** creates a kinematic event chain where the movement represented by the class object is prepended to the argument _to_add */
  CombinedKinematicMovementDescription operator+( KinematicMovementDescription const& _to_add );
  
  /** creates a kinematic event chain where the movement represented by the class object is prepended to the argument _to_add */
  CombinedKinematicMovementDescription operator+( CombinedKinematicMovementDescription const& _to_add );
  
  
private:
  boost::shared_ptr<RelativeMovementInstance> enwrapped_relative_movement_;
};

typedef RelativeMovement RelMove;

/** class for for relative movements that can be applied to a base position */
class RelativeMovement::RelativeMovementInstance
{
public:
  /** returns the type of the relative movement */
  virtual std::string type()=0;
  
  /** applies the relative movement to a base pose */
  virtual movements::Pose applyToBasePose( movements::Pose const& _base )=0;
};

}