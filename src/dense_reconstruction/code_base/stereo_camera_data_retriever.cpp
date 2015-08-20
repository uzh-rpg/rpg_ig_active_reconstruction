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

#include "dense_reconstruction/stereo_camera_data_retriever.h"

namespace dense_reconstruction
{


StereoCameraDataRetriever::StereoCameraDataRetriever( YoubotPlanner* _robot_interface, std::string _youbot_interface_namespace )
  :republish_(false)
{
  std::string  octomap_topic, pcl_in_topic, pcl_out_topic;
  if( !ros::param::get("/"+_youbot_interface_namespace+"/initialization/stereo_camera/octomap_topic",octomap_topic) )
  {
    ROS_WARN_STREAM("StereoCameraDataRetriever:: No pointcloud topic was found on parameter server ('"<<"/"+_youbot_interface_namespace+"/initialization/stereo_camera/octomap_topic"<<"'), the default ('/octomap_full') will be used.");
    octomap_topic="/octomap_full";
  }
  if( !ros::param::get("/"+_youbot_interface_namespace+"/initialization/stereo_camera/pcl_in_topic",pcl_in_topic) )
  {
    ROS_WARN_STREAM("StereoCameraDataRetriever:: No pointcloud topic was found on parameter server ('"<<"/"+_youbot_interface_namespace+"/initialization/stereo_camera/pcl_in_topic"<<"'), the default ('/camera/points2') will be used.");
    pcl_in_topic="/camera/points2";
  }
  if( !ros::param::get("/"+_youbot_interface_namespace+"/initialization/stereo_camera/pcl_out_topic",pcl_out_topic) )
  {
    ROS_WARN_STREAM("StereoCameraDataRetriever:: No pointcloud topic was found on parameter server ('"<<"/"+_youbot_interface_namespace+"/initialization/stereo_camera/pcl_out_topic"<<"'), the default ('/remode/pointcloud_single') will be used.");
    pcl_out_topic="/remode/pointcloud_single";
  }
  double max_wait_time;
  if( !ros::param::get("/"+_youbot_interface_namespace+"/initialization/stereo_camera/max_pcl_wait_time",max_wait_time) )
  {
    ROS_WARN_STREAM("StereoCameraDataRetriever:: No max_wait_time was found on parameter server ('"<<"/"+_youbot_interface_namespace+"/initialization/stereo_camera/max_wait_time"<<"'), the default (3s) will be used.");
    max_pointcloud_wait_time_=ros::Duration(3.0); // 3 seconds
  }
  else
  {
    max_pointcloud_wait_time_=ros::Duration(max_wait_time); // [s]
  }
  double max_octomap_wait_time;
  if( !ros::param::get("/"+_youbot_interface_namespace+"/initialization/stereo_camera/max_octomap_wait_time",max_octomap_wait_time) )
  {
    ROS_WARN_STREAM("StereoCameraDataRetriever:: No max_octomap_wait_time was found on parameter server ('"<<"/"+_youbot_interface_namespace+"/initialization/stereo_camera/max_octomap_wait_time"<<"'), the default (10s) will be used.");
    max_octomap_wait_time_=ros::Duration(10.0); // 3 seconds
  }
  else
  {
    max_octomap_wait_time_=ros::Duration(max_octomap_wait_time); // [s]
  }
  
  octomap_has_published_=false;
  
  ros::NodeHandle nh;
  
  octomap_topic_subscriber_ = nh.subscribe( octomap_topic,1, &dense_reconstruction::StereoCameraDataRetriever::octomapCallback, this );
  pcl_subscriber_ = nh.subscribe( pcl_in_topic,1, &dense_reconstruction::StereoCameraDataRetriever::pclCallback, this );
  pcl_publisher_ = nh.advertise<sensor_msgs::PointCloud2>( pcl_out_topic, 1 );
}

std::string StereoCameraDataRetriever::movementConfigurationDescription()
{
  std::stringstream name;
  name<<"Static";
  return name.str();
}

RobotPlanningInterface::ReceiveInfo StereoCameraDataRetriever::retrieveData()
{
  /*if( robot_interface_->current_view_==nullptr )
  {
    // TODO in unknown pose: replan!
    ROS_WARN("YoubotPlanner::StereoCameraDataRetriever::retrieveData::Retrieving data but the view seems to be unknown, might have problems with the poses.");
  }*/
  
  
  // get pointcloud from topic and republish it to octomap
  ros::Time max = ros::Time::now() + max_pointcloud_wait_time_;
  
  octomap_has_published_ = false;
  republish_ = true;
  ros::spinOnce();
  
  while( republish_== true )
  {
    ros::Duration(0.005).sleep();
    ros::spinOnce();
    
    if( ros::Time::now()>max )
    {
      ROS_WARN("StereoCameraDataRetriever::retrieveData:: Didn't get any PCL input in time.");
      return RobotPlanningInterface::RECEPTION_FAILED;
    }
  }
  
  max = ros::Time::now() + max_octomap_wait_time_;
  
  while( !octomap_has_published_ )
  {
    ros::Duration(0.005).sleep();
    ros::spinOnce();
    
    if( ros::Time::now()>max )
    {
      ROS_WARN("StereoCameraDataRetriever::retrieveData:: Octomap didn't publish data in time.");
      return RobotPlanningInterface::RECEPTION_FAILED;
    }
  }
    
  return RobotPlanningInterface::RECEIVED;
}

bool StereoCameraDataRetriever::movementNeeded()
{
  return false;
}

bool StereoCameraDataRetriever::getRetrievalMovement( robot_state::RobotState& _state, movements::KinematicMovementDescription* _retrieval_movement, movements::KinematicMovementDescription::PathInfo* _additional_info )
{
  
  return false;
}

void StereoCameraDataRetriever::pclCallback( const sensor_msgs::PointCloud2ConstPtr& _msg )
{
  if( republish_ )
  {
    pcl_publisher_.publish(_msg);
    republish_ = false;
  }
}

void StereoCameraDataRetriever::octomapCallback( const octomap_msgs::OctomapConstPtr& _msg )
{
  octomap_has_published_ = true;
}

}