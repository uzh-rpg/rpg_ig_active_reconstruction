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

/// set of explicit conversion functions between ros and st_is

#include "geometry_msgs/Pose.h"
#include "movements/geometry_pose.h"

namespace movements
{

geometry_msgs::Pose toROS( movements::Pose _pose );
movements::Pose fromROS( geometry_msgs::Pose _pose );
std::vector<geometry_msgs::Pose> toROS( movements::PoseVector _to_convert );
movements::PoseVector fromROS( std::vector<geometry_msgs::Pose> _to_convert );

}