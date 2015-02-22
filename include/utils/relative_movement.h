/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
* 
relative_movement is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
relative_movement is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with relative_movement. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "utils/geometry_pose.h"
#include <boost/shared_ptr.hpp>

namespace st_is
{
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
  st_is::GeometryPose applyToBasePose( st_is::GeometryPose& _base );
  
  /** returns a pointer to the internally hold RelativeMovementInstance */
  boost::shared_ptr<RelativeMovementInstance> operator*();
private:
  boost::shared_ptr<RelativeMovementInstance> enwrapped_relative_movement_;
};

/** class for for relative movements that can be applied to a base position */
class RelativeMovement::RelativeMovementInstance
{
  /** returns the type of the relative movement */
  std::string type()=0;
  
  /** applies the relative movement to a base pose */
  st_is::GeometryPose applyToBasePose( st_is::GeometryPose& _base )=0;
};

}