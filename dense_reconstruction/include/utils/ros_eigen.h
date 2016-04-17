 /* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*

ros_eigen is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
ros_eigen is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with ros_eigen. If not, see <http://www.gnu.org/licenses/>.
*/ 

/// set of convenience functions and classes when working with Eigen in ROS (e.g. conversions between types)

#pragma once

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Transform.h"

namespace st_is
{

Eigen::Vector3d geometryToEigen( const geometry_msgs::Point& _vec );
Eigen::Vector3d geometryToEigen( const geometry_msgs::Vector3& _vec );
Eigen::Quaterniond geometryToEigen( const geometry_msgs::Quaternion& _quat );

geometry_msgs::Point eigenToGeometry( const Eigen::Vector3d& _vec );
geometry_msgs::Quaternion eigenToGeometry( const Eigen::Quaterniond& _quat );

/// returns the transformation matrix represented by _pose
Eigen::Matrix<double,3,4> transformationMatrix( geometry_msgs::Pose& _pose );
Eigen::Matrix<double,3,4> transformationMatrix( geometry_msgs::Transform& _pose );
geometry_msgs::Pose geometryPose( Eigen::Matrix<double,3,4>& _pose );
geometry_msgs::Pose geometryPose( Eigen::Matrix<double,4,4>& _pose );
    
}