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

#include "dense_reconstruction/remode_feeder.h"
#include <movements/ros_movements.h>
#include "utils/ros_eigen.h"
#include <tf/transform_broadcaster.h>
#include <boost/foreach.hpp>

namespace dense_reconstruction
{

RemodeFeeder::RemodeFeeder( ros::NodeHandle& _n, unsigned int _publish_every_xth_frame ):
  nh_(_n),
  tf_listener_(_n),
  publish_every_xth_frame_(_publish_every_xth_frame),
  publish_count_(0)
{
  world_frame_ = "dr_origin";
  std::string remode_input_topic = "/dense_reconstruction/remode_feed";
  std::string svo_topic = "/svo/dense_input";
  
  
  camera_frame_ = "camera";
  std::string image_topic = "/camera/image_rect";
  
  min_depth_ = 0.1;
  max_depth_ = 1.5;
  
  //image_stream_ = nh_.subscribe( image_topic,1 ,&RemodeFeeder::imageStreamCallback, this ); // first try version
  feeder_ = nh_.advertise<svo_msgs::DenseInputWithFeatures>( remode_input_topic,10 );
  svo_subscriber_ = nh_.subscribe( svo_topic,1 , &RemodeFeeder::svoCallback, this );
  
  // to get poses directly from gazebo
  /*got_gazebo_pose_ = false;
  ros::Subscriber gazebo = nh_.subscribe( "/gazebo/link_states",1 ,&RemodeFeeder::gazeboCallback, this );*/
}

void RemodeFeeder::imageStreamCallback(  const sensor_msgs::ImageConstPtr& _newImage )
{
  if( publish_count_==publish_every_xth_frame_ )
  {
    publish_count_ = 0;
  }
  publish_count_++;
  if( publish_count_!=1 )
  {
    return;
  }
  
    
  
  ros::Time image_time = _newImage->header.stamp;
  movements::Pose camera_pose;
  
  if( poseFromTF( camera_frame_, world_frame_, image_time, &camera_pose, 3.0 ) )
  {
    // pose was found
    svo_msgs::DenseInputWithFeatures msg;
    
    // if you want to get a pose from gazebo directly (cam is not included though)
    /*if( got_gazebo_pose_ )
      camera_pose = groundTruthStateFromGazebo();*/
    
    msg.header = _newImage->header;
    msg.header.frame_id = world_frame_;
    msg.pose = movements::toROS(camera_pose);
    msg.image = *_newImage;
    msg.min_depth = min_depth_;
    msg.max_depth = max_depth_;
    
    feeder_.publish(msg);
  }
  // else do nothing
}

bool RemodeFeeder::poseFromTF( std::string _source, std::string _target, ros::Time _time, movements::Pose* _output, double _max_wait_time )
{
  
  bool new_tf_available = tf_listener_.waitForTransform( _target,_source, _time, ros::Duration(_max_wait_time) );
  
  if( !new_tf_available )
    return false;
  
  // set world and odom to have the same origin
  //static tf::TransformBroadcaster br;
  //br.sendTransform(tf::StampedTransform(tf::Transform(), ros::Time::now(), "world", "odom"));
  
  tf::StampedTransform pose;
  geometry_msgs::TransformStamped end_effector_;
  
  try{
    tf_listener_.lookupTransform(_target, _source, _time, pose);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return false;
  }
  
  tf::transformStampedTFToMsg( pose, end_effector_ );
  
  Eigen::Quaterniond rotation = st_is::geometryToEigen( end_effector_.transform.rotation );
  Eigen::Vector3d translation = st_is::geometryToEigen( end_effector_.transform.translation );
  
  *_output = movements::Pose( translation, rotation );
  return true;
}

void RemodeFeeder::gazeboCallback( const gazebo_msgs::LinkStatesConstPtr& _gazebo_states )
{
  got_gazebo_pose_ = true;
  last_gazebo_msg_ = *_gazebo_states;
}

void RemodeFeeder::svoCallback( const svo_msgs::DenseInputWithFeaturesConstPtr& _svo_output )
{
  ros::Time now = _svo_output->header.stamp;
  bool ground_tf_available = tf_listener_.waitForTransform( "dr_origin","world",now,ros::Duration(0.03) );
  
  if(ground_tf_available) // transform here because it would be much more overhead to do it for every point in the point cloud remote publishes
  {
    tf::StampedTransform t_OW; // world to origin
    tf_listener_.lookupTransform("dr_origin","world",now,t_OW);
    
    svo_msgs::DenseInputWithFeatures msg = *_svo_output;
    
    // set frame in header
    msg.header.frame_id = world_frame_;
    
    // pose: P_OC = P_OW*P_WC
    tf::Transform t_WC; //cam_pos to world
    tf::poseMsgToTF( msg.pose,t_WC );
    tf::Transform t_OC = t_OW*t_WC;
    tf::poseTFToMsg( t_OC, msg.pose );
    // image unchanged
    // min_depth unchanged
    // max_depth unchanged
    BOOST_FOREACH( auto feature, msg.features )
    {
      tf::Vector3 feat_W(feature.x,feature.y,feature.z);
      tf::Vector3 feat_O = t_OW*feat_W;
      feature.x = feat_O.x();
      feature.y = feat_O.y();
      feature.z = feat_O.z();
    }
    //bgr_image unchanged
    feeder_.publish(msg);
  }
  else
  {
    ROS_WARN("Couldn't find transformation from 'dr_origin' to 'world'.");
  }
  
}

movements::Pose RemodeFeeder::groundTruthStateFromGazebo()
{
  // actually would have to read up if this really isn't affected by noise - but it's unused anyway
  int cam_index = 0;
  for( ; cam_index<last_gazebo_msg_.name.size(); cam_index++ )
  {
    if( last_gazebo_msg_.name[cam_index]=="youbot::arm_link_5" ) //camera ain't there, don't know why
    {
      break;
    }
  }
  return movements::fromROS( last_gazebo_msg_.pose[cam_index] );
}


}