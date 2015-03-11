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
#include "geometry_msgs/Transform.h"

#include <boost/thread/mutex.hpp>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "dense_reconstruction/PoseSetter.h"
#include "dense_reconstruction/IsOk.h"

namespace
dense_reconstruction{

class TFLinker
{
public:
  TFLinker( ros::NodeHandle _nh, ros::Duration _max_svo_wait_time );
  
  /** on iteration */
  void run();
  
  bool setWorldPoseRequest( dense_reconstruction::PoseSetter::Request& _req, dense_reconstruction::PoseSetter::Response& _res );
  /** whether the published tf tree is up to date */
  bool tfUpToDate( dense_reconstruction::IsOk::Request& _req, dense_reconstruction::IsOk::Response& _res );
  
  /** attempts to load hand eye transformation data from parameter server */
  bool loadHEC();
private:
  ros::NodeHandle nh_;
  // servers
  ros::ServiceServer tree_connector_;
  ros::ServiceServer status_answers_;
  
  // tf broadcaster & listener
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  
  
  boost::mutex pose_protector_;
  tf::Transform world2dr_origin_; // as data is transformed
  tf::Transform odom2dr_origin_;
  tf::Transform arm2image_; // from hec

  bool tf_up_to_date_;
  ros::Duration max_tf_wait_time_; // max time to wait for all needed tfs to be available before tf state is set to be invalid

};

TFLinker::TFLinker( ros::NodeHandle _nh, ros::Duration _max_svo_wait_time )
  : nh_(_nh)
  , max_tf_wait_time_(_max_svo_wait_time)
  , tf_up_to_date_(false)
{
  // initialize transforms
  world2dr_origin_.setIdentity();
  odom2dr_origin_.setIdentity();
  if( !loadHEC() )
  {
    ROS_FATAL("TFLinker::TFLinker::Hand-eye transformationn under '/hec/arm2image' was not properly configured. Shutting down node.");
    ros::shutdown();
  }
  
  // setup servers
  tree_connector_ = nh_.advertiseService("dense_reconstruction/set_world_pose", &TFLinker::setWorldPoseRequest, this );
  status_answers_ = nh_.advertiseService("dense_reconstruction/svo_pose_available", &TFLinker::tfUpToDate, this );
}

void TFLinker::run()
{
  tf::Transform world2dr_origin;
  { // allows for multithreaded spinning
    boost::mutex::scoped_lock scoped_lock(pose_protector_);
    world2dr_origin = world2dr_origin_;
  }
  
  // get newest robot transform
  ros::Time now = ros::Time::now();
  
  bool in_time = tf_listener_.waitForTransform( "arm_link_5", "base_footprint", now, max_tf_wait_time_ );
  
  if( !in_time )
  {
    ROS_WARN("TFLinker:: Couldn't get current tf transform from 'base_footprint' to 'arm_link_5' in time.");
    tf_up_to_date_ = false;
    return;
  }
  
  tf::StampedTransform base2arm_link_5;
  tf_listener_.lookupTransform("arm_link_5", "base_footprint", now, base2arm_link_5);
  
  // build new complete robot transfrom
  tf::Transform base2image = arm2image_*base2arm_link_5;
  
  // broadcast new transforms
  tf_broadcaster_.sendTransform(tf::StampedTransform(base2image, now, "cam_pos", "youbot_base"));
  tf_broadcaster_.sendTransform(tf::StampedTransform(world2dr_origin_, now, "dr_origin", "world"));
  
  tf_up_to_date_ = true;
  
  return;
}

bool TFLinker::setWorldPoseRequest( dense_reconstruction::PoseSetter::Request& _req, dense_reconstruction::PoseSetter::Response& _res )
{
  geometry_msgs::Transform new_world_pose = _req.pose;
  
  { // allows for multithreaded spinning
    boost::mutex::scoped_lock scoped_lock(pose_protector_);
    tf::transformMsgToTF( new_world_pose, world2dr_origin_ );
  }
  _res.success = true;
  return true;
}

bool TFLinker::tfUpToDate( dense_reconstruction::IsOk::Request& _req, dense_reconstruction::IsOk::Response& _res )
{
  _res.is_ok = tf_up_to_date_;
  
  return true;
}

bool TFLinker::loadHEC()
{
  geometry_msgs::Transform arm2image;
  
  bool success = nh_.getParam("/hec/arm2image/translation/x", arm2image.translation.x )
              && nh_.getParam("/hec/arm2image/translation/y", arm2image.translation.y )
              && nh_.getParam("/hec/arm2image/translation/z", arm2image.translation.z )
              && nh_.getParam("/hec/arm2image/rotation/x", arm2image.rotation.x )
              && nh_.getParam("/hec/arm2image/rotation/y", arm2image.rotation.y )
              && nh_.getParam("/hec/arm2image/rotation/z", arm2image.rotation.z )
              && nh_.getParam("/hec/arm2image/rotation/w", arm2image.rotation.w );
  if( !success )
  {
    return true;
  }
  
  tf::transformMsgToTF( arm2image, arm2image_ );
  
  return true;
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_linker");
  ros::NodeHandle n;
  
  dense_reconstruction::TFLinker tfl( n, ros::Duration(0.03) );
  
  
  
  ROS_INFO("Starting TF Linker.");
  ros::Rate rate(10);
  
  while ( n.ok() )
  {
    tfl.run();
    
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
} 
