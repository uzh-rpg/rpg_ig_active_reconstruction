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

#include "utils/ros_eigen.h"

namespace st_is
{

Eigen::Vector3d geometryToEigen( const geometry_msgs::Point& _vec )
{
  Eigen::Vector3d output;
  output.x() = _vec.x;
  output.y() = _vec.y;
  output.z() = _vec.z;
  return output;
}


Eigen::Vector3d geometryToEigen( const geometry_msgs::Vector3& _vec )
{
  Eigen::Vector3d output;
  output.x() = _vec.x;
  output.y() = _vec.y;
  output.z() = _vec.z;
  return output;
}


Eigen::Quaterniond geometryToEigen( const geometry_msgs::Quaternion& _quat )
{// Eigen returns the rotation quaternion that would correspond to rotating the coordinate system, not the points/vectors, thus the transposition is necessary
  Eigen::Quaterniond output;
  output.x() = _quat.x;
  output.y() = _quat.y;
  output.z() = _quat.z;
  output.w() = _quat.w;
  return output;
}


geometry_msgs::Point eigenToGeometry( const Eigen::Vector3d& _vec )
{
  geometry_msgs::Point output;
  output.x = _vec.x();
  output.y = _vec.y();
  output.z = _vec.z();
  return output;
}


geometry_msgs::Quaternion eigenToGeometry( const Eigen::Quaterniond& _quat )
{// Eigen returns the rotation quaternion that would correspond to rotating the coordinate system, not the points/vectors, thus the transposition is necessary
  geometry_msgs::Quaternion output;
  output.x = _quat.x();
  output.y = _quat.y();
  output.z = _quat.z();
  output.w = _quat.w();
  return output;
}


Eigen::Matrix<double,3,4> transformationMatrix( geometry_msgs::Pose& _pose )
{
  Eigen::Matrix<double,3,4> transformation_matrix;
  Eigen::Vector3d translation = geometryToEigen( _pose.position );
  Eigen::Quaterniond rotation = geometryToEigen( _pose.orientation );
  
  transformation_matrix.leftCols<3>() = rotation.toRotationMatrix();
  transformation_matrix.rightCols<1>() = translation;
  
  return transformation_matrix;
}

geometry_msgs::Pose geometryPose( Eigen::Matrix<double,3,4>& _pose )
{
  Eigen::Quaterniond rotation( _pose.leftCols<3>() );
  /*using namespace std;
  cout<<endl<<"temporary quaternion source matrix:";;
  cout<<endl<<_pose.leftCols<3>();
  cout<<endl<<"temporary quaternion:";
  cout<<endl<<"x:"<<rotation.x();
  cout<<endl<<"y:"<<rotation.y();
  cout<<endl<<"z:"<<rotation.z();
  cout<<endl<<"w:"<<rotation.w();
  cout<<endl<<"which still represents the rotation matrix:";
  cout<<endl<<rotation.matrix();*/
  geometry_msgs::Pose pose;
  pose.orientation = eigenToGeometry( rotation );
  pose.position = eigenToGeometry( _pose.rightCols<1>() );
  return pose;
}

geometry_msgs::Pose geometryPose( Eigen::Matrix<double,4,4>& _pose )
{
  Eigen::Quaterniond rotation( _pose.topLeftCorner<3,3>() );
  geometry_msgs::Pose pose;
  pose.orientation = eigenToGeometry( rotation );
  pose.position = eigenToGeometry( _pose.topRightCorner<3,1>() );
  return pose;
}

}