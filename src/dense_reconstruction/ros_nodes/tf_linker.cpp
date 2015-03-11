/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of dense_reconstruction, a ROS package for...well,

dense_reconstruction is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
dense_reconstruction is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with dense_reconstruction. If not, see <http://www.gnu.org/licenses/>.
*/

/** transmits additional tf transforms that are needed, connecting
 * the two independent trees: the robot tree (from Youbot or Gazebo) and the SVO tree
 * Needs hand eye calibration data which it loads from parameter server
 * 
 */

#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

#include <boost/thread/mutex.hpp>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "dense_reconstruction/PoseSetter.h"
#include "dense_reconstruction/IsOk.h"


boost::mutex pose_protector;
tf::Transfrom world2dr_origin;
tf::Transfrom odom2dr_origin;

bool new_cam_pose_available;
ros::Time last_update;

bool setPoseRequest( dense_reconstruction::PoseSetter::Request& _req, dense_reconstruction::PoseSetter::Response& _res )
{
  return true;
}

bool svoAvailableRequest( dense_reconstruction::IsOk::Request& _req, dense_reconstruction::IsOk::Response& _res )
{
  return true;
}

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_linker");
  ros::NodeHandle n;
  
  // initialize transforms
  world2dr_origin.setIdentity();
  odom2dr_origin.setIdentity();
  
  // setup server
  ros::ServiceServer tree_connector;
  tree_connector = n.advertiseService("dense_reconstruction/set_world_pose", setPoseRequest );
  ros::ServiceServer status_answers;
  status_answers = n.advertiseService("dense_reconstruction/svo_pose_available", svoAvailableRequest );
  
  ros::Rate rate(10);
  
  while ( n.ok() )
  {
    cout<<endl<<"Working: "<<endl;
  }
  
  return 0;
} 
