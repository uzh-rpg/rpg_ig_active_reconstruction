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

#include "utils/relative_movement.h"

RelativeMovement::RelativeMovement( RelativeMovementInstance* _to_enwrap )
{
  enwrapped_relative_movement_ = boost::shared_ptr<RelativeMovementInstance>( _to_enwrap );
}

std::string RelativeMovement::type()
{
  return enwrapped_relative_movement_->type();
}

st_is::GeometryPose RelativeMovement::applyToBasePose( st_is::GeometryPose& _base )
{
  return enwrapped_relative_movement_->applyToBasePose(_base);
}

boost::shared_ptr<RelativeMovementInstance> RelativeMovement::operator*()
{
  return enwrapped_relative_movement_;
}