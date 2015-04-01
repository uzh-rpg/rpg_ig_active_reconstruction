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

#include <gazebo_msgs/ModelStates.h>
#include "dense_reconstruction/SetScale.h"
#include <tf/tfMessage.h>

namespace
dense_reconstruction{

class TFLinker
{
public:
  TFLinker( ros::NodeHandle _nh, ros::Duration _max_svo_wait_time );
  
  /** on iteration */
  void publish();
  
  /** expects the transform from dr_origin coordinates into world coordinates */
  bool setWorldPoseRequest( dense_reconstruction::PoseSetter::Request& _req, dense_reconstruction::PoseSetter::Response& _res );
  /** whether the published tf tree is up to date */
  bool tfUpToDate( dense_reconstruction::IsOk::Request& _req, dense_reconstruction::IsOk::Response& _res );
  
  /** attempts to load hand eye transformation data from parameter server */
  bool loadHEC();
  
  void modelStateCallback( const gazebo_msgs::ModelStatesConstPtr& _msg );
  void tfCallback( const tf::tfMessageConstPtr& _msg );
  bool setSVOScaleService( SetScale::Request& _req, SetScale::Response& _res );
private:
  ros::NodeHandle nh_;
  // servers
  ros::ServiceServer tree_connector_;
  ros::ServiceServer status_answers_;
  ros::ServiceServer set_svo_scale_server_;
  
  ros::Subscriber gazebo_state_;
  ros::Subscriber tf_subscriber_;
  
  // tf broadcaster & listener
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  
  double svo_scale_;
  bool use_gazebo_ground_truth_;
  bool dr_origin_is_odom_; // if odometry is to be used as base
  bool is_setup_; //only starts publishing when it receveived at least on world pose
  boost::mutex pose_protector_;
  tf::Transform dr_origin2world_; // as data is transformed
  tf::Transform odom2dr_origin_;
  tf::Transform image2arm_; // from hec

  bool tf_up_to_date_;
  ros::Duration max_tf_wait_time_; // max time to wait for all needed tfs to be available before tf state is set to be invalid

};

TFLinker::TFLinker( ros::NodeHandle _nh, ros::Duration _max_svo_wait_time )
  : nh_(_nh)
  , max_tf_wait_time_(_max_svo_wait_time)
  , tf_up_to_date_(false)
  , tf_listener_(ros::Duration(30.0))
  , is_setup_(false)
  , use_gazebo_ground_truth_(false)
  , svo_scale_(1.0)
{
  // initialize transforms
  dr_origin2world_.setIdentity();
  odom2dr_origin_.setIdentity();
  if( !loadHEC() )
  {
    ROS_FATAL("TFLinker::TFLinker::Hand-eye transformationn under '/hec/arm2image' was not properly configured. Shutting down node.");
    ros::shutdown();
  }
  ros::param::get("/use_gazebo_ground_truth", use_gazebo_ground_truth_);
  
  // setup servers
  tree_connector_ = nh_.advertiseService("/dense_reconstruction/set_world_pose", &TFLinker::setWorldPoseRequest, this );
  status_answers_ = nh_.advertiseService("/dense_reconstruction/svo_pose_available", &TFLinker::tfUpToDate, this );
  set_svo_scale_server_ = nh_.advertiseService("dense_reconstruction/tf_linker/set_svo_scale", &TFLinker::setSVOScaleService, this );
  
  if( use_gazebo_ground_truth_ )
    gazebo_state_ = nh_.subscribe("/gazebo/model_states",1,&dense_reconstruction::TFLinker::modelStateCallback, this );
  else
    tf_subscriber_ = nh_.subscribe("/tf",100,&TFLinker::tfCallback, this );
}

void TFLinker::publish()
{
  if(!is_setup_)
  {
    if(!ros::param::get("/youbot_interface/setup_tf_for_svo", dr_origin_is_odom_ ) )
    {
      return;
    }
    else
    {
      dr_origin_is_odom_ = !dr_origin_is_odom_;
    }
    ros::Time now = ros::Time::now();
    tf_broadcaster_.sendTransform(tf::StampedTransform(image2arm_, now, "arm_link_5", "cam_pos"));
    return;
  }
  else
  {
    dr_origin_is_odom_ = false;
  }
  
  tf::Transform dr_origin2world;
  { // allows for multithreaded spinning
    boost::mutex::scoped_lock scoped_lock(pose_protector_);
    dr_origin2world = dr_origin2world_;
  }
  ros::Time now = ros::Time::now();
  if( use_gazebo_ground_truth_ )
  {
    // callback does that
  }
  else if( dr_origin_is_odom_ )
  {
    tf_broadcaster_.sendTransform(tf::StampedTransform(dr_origin2world, now, "odom", "dr_origin")); // dr origin to world is the identity if not set otherwise
  }
  /*else // tf with svo
  {
    ROS_INFO("Publishing!");
    tf::Transform t_WG = dr_origin2world;
    bool in_time = tf_listener_.waitForTransform( "cam_pos", "world", now, max_tf_wait_time_ );
    if( !in_time )
    {
      ROS_WARN("TFLinker:: Couldn't get current tf transform from 'world' to 'cam_pos' in time.");
      return;
    }
    tf::StampedTransform t_CW;
    tf_listener_.lookupTransform("cam_pos", "world", now, t_CW);
    tf::Vector3 trans_CW = svo_scale_*t_CW.getOrigin();
    t_CW.setOrigin(trans_CW);
    
    tf::Transform t_CG = t_CW*t_WG;
    tf_broadcaster_.sendTransform(tf::StampedTransform(t_CG, now, "cam_pos", "dr_origin"));
  }*/
  
  tf_broadcaster_.sendTransform(tf::StampedTransform(image2arm_, now, "arm_link_5", "cam_pos"));
  /*
  // get newest robot transform
  ros::Time now = ros::Time::now();
  // TODO:check timing constraints, is the last issued pose good enough?
  bool in_time = tf_listener_.waitForTransform( "arm_link_5", "base_footprint", now, max_tf_wait_time_ );
  bool in_time2 = tf_listener_.waitForTransform( "world", "cam_pos", now, max_tf_wait_time_ );
  
  if( !in_time || !in_time2 )
  {
    ROS_WARN("TFLinker:: Couldn't get current tf transform from 'base_footprint' to 'arm_link_5' in time.");
    tf_up_to_date_ = false;
    return;
  }
  
  tf::StampedTransform base2arm_link_5;
  tf_listener_.lookupTransform("arm_link_5", "base_footprint", now, base2arm_link_5);
  
  // build new complete robot transfrom
  tf::Transform base2image = arm2image_*base2arm_link_5;
  
  // t_OC = t_OW*t_WC
  tf::StampedTransform t_WC;
  tf_listener_.lookupTransform("world", "cam_pos", now, t_WC);
  tf::Transform t_OC = world2dr_origin_*t_WC;
  
  // broadcast new transforms
  tf_broadcaster_.sendTransform(tf::StampedTransform(base2image, now, "cam_pos", "youbot_base")); //ok
  tf_broadcaster_.sendTransform(tf::StampedTransform(t_OC, now, "dr_origin", "cam_pos"));
  
  // needs to become: tf_broadcaster_.sendTransform(tf::StampedTransform(xxx, now, "dr_origin", "base_link"));
  tf_broadcaster_.sendTransform(tf::StampedTransform(odom2dr_origin_, now, "dr_origin", "odom"));
  */
  tf_up_to_date_ = true;
  
  return;
}

bool TFLinker::setWorldPoseRequest( dense_reconstruction::PoseSetter::Request& _req, dense_reconstruction::PoseSetter::Response& _res )
{
  geometry_msgs::Transform new_world_pose = _req.pose;
  
  { // allows for multithreaded spinning
    boost::mutex::scoped_lock scoped_lock(pose_protector_);
    tf::transformMsgToTF( new_world_pose, dr_origin2world_ );
  }
  ROS_INFO("Setting world pose.");
  _res.success = true;
  is_setup_ = true;
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
  
  tf::Transform arm2image_tf;
  tf::transformMsgToTF( arm2image, arm2image_tf );
  
  image2arm_ = arm2image_tf.inverse();
  return true;
}

// ok, this is a little specific, but hey... it's got to run now!
void TFLinker::modelStateCallback( const gazebo_msgs::ModelStatesConstPtr& _msg )
{
  // if something is published here, then the model is in simulation...
  
  
  unsigned int robot_idx;
  for( unsigned int i=0; i<_msg->name.size(); ++i)
  {
    if( _msg->name[i]=="youbot" )
    {
      robot_idx = i;
    }
    if( i==_msg->name.size()-1 && robot_idx!=i ) // couldn't find robot name in array
    {
      return;
    }
  }
  
  geometry_msgs::Pose robot_pose_ground_truth = _msg->pose[robot_idx];
  geometry_msgs::Transform rp_gt;
  rp_gt.translation.x = robot_pose_ground_truth.position.x;
  rp_gt.translation.y = robot_pose_ground_truth.position.y;
  rp_gt.translation.z = robot_pose_ground_truth.position.z;
  rp_gt.rotation.x = robot_pose_ground_truth.orientation.x;
  rp_gt.rotation.y = robot_pose_ground_truth.orientation.y;
  rp_gt.rotation.z = robot_pose_ground_truth.orientation.z;
  rp_gt.rotation.w = robot_pose_ground_truth.orientation.w;
  
  tf::Transform t_GR;
  tf::transformMsgToTF( rp_gt, t_GR );
  
  tf_broadcaster_.sendTransform(tf::StampedTransform(t_GR, ros::Time::now(), "dr_origin", "base_footprint"));
  
}

void TFLinker::tfCallback( const tf::tfMessageConstPtr& _msg )
{
  if( use_gazebo_ground_truth_ || dr_origin_is_odom_ || !is_setup_ )
  {
    ROS_INFO("Exiting for lack of information.");
    return;
  }
  
  if( _msg->transforms[0].header.frame_id=="cam_pos" && _msg->transforms[0].child_frame_id=="world" )
  {
    tf::StampedTransform t_CW;
    transformMsgToTF( _msg->transforms[0].transform, t_CW );
    tf::Vector3 trans_CW = svo_scale_*t_CW.getOrigin();
    t_CW.setOrigin(trans_CW);
    
    tf::Transform t_WG = dr_origin2world_;
    tf::Transform t_CG = t_CW*t_WG;
    tf_broadcaster_.sendTransform(tf::StampedTransform(t_CG, _msg->transforms[0].header.stamp, "cam_pos", "dr_origin"));
  }
}

bool TFLinker::setSVOScaleService( SetScale::Request& _req, SetScale::Response& _res )
{
  ROS_INFO_STREAM("TFLinker::setSVOScaleService:: Setting new SVO scale: "<<svo_scale_<<".");
  svo_scale_ = _req.scale;
  return true;
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_linker");
  ros::NodeHandle n;
  
  dense_reconstruction::TFLinker tfl( n, ros::Duration(0.06) );
  
  
  
  ROS_INFO("Starting TF Linker.");
  ros::Rate rate(50);
  
  while ( n.ok() )
  {
    tfl.publish();
    
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
} 
