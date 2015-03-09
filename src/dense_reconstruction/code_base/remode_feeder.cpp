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

namespace dense_reconstruction
{

RemodeFeeder::RemodeFeeder( ros::NodeHandle& _n, unsigned int _publish_every_xth_frame ):
  nh_(_n),
  tf_listener_(_n),
  publish_every_xth_frame_(_publish_every_xth_frame),
  publish_count_(0)
{
  world_frame_ = "odom";
  camera_frame_ = "camera";
  std::string image_topic = "youbot/eye/image_rect";
  std::string remode_input_topic = "dense_reconstruction/remode_feed";
  
  min_depth_ = 0.1;
  max_depth_ = 1.5;
  
  image_stream_ = nh_.subscribe( image_topic,1 ,&RemodeFeeder::imageStreamCallback, this );
  feeder_ = nh_.advertise<svo_msgs::DenseInputWithFeatures>( remode_input_topic,10 );
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


}