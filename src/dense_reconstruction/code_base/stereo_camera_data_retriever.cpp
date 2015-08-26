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
#include <rosbag/bag.h>

namespace dense_reconstruction
{


StereoCameraDataRetriever::StereoCameraDataRetriever( std::string interface_namespace )
  :republish_(false)
  ,wait_for_octomap_(true)
  ,dataCount_(0)
{
  std::string  octomap_topic, pcl_out_topic;
  if( !ros::param::get("/"+interface_namespace+"/initialization/stereo_camera/octomap_topic",octomap_topic) )
  {
    ROS_WARN_STREAM("StereoCameraDataRetriever:: No pointcloud topic was found on parameter server ('"<<"/"+interface_namespace+"/initialization/stereo_camera/octomap_topic"<<"'), the default ('/octomap_full') will be used.");
    octomap_topic="/octomap_full";
  }
  if( !ros::param::get("/"+interface_namespace+"/initialization/stereo_camera/pcl_in_topic_",pcl_in_topic_) )
  {
    ROS_WARN_STREAM("StereoCameraDataRetriever:: No pointcloud topic was found on parameter server ('"<<"/"+interface_namespace+"/initialization/stereo_camera/pcl_in_topic_"<<"'), the default ('/camera/points2') will be used.");
    pcl_in_topic_="/camera/points2";
  }
  if( !ros::param::get("/"+interface_namespace+"/initialization/stereo_camera/pcl_out_topic",pcl_out_topic) )
  {
    ROS_WARN_STREAM("StereoCameraDataRetriever:: No pointcloud topic was found on parameter server ('"<<"/"+interface_namespace+"/initialization/stereo_camera/pcl_out_topic"<<"'), the default ('/remode/pointcloud_single') will be used.");
    pcl_out_topic="/remode/pointcloud_single";
  }
  double max_wait_time;
  if( !ros::param::get("/"+interface_namespace+"/initialization/stereo_camera/max_pcl_wait_time",max_wait_time) )
  {
    ROS_WARN_STREAM("StereoCameraDataRetriever:: No max_wait_time was found on parameter server ('"<<"/"+interface_namespace+"/initialization/stereo_camera/max_wait_time"<<"'), the default (3s) will be used.");
    max_pointcloud_wait_time_=ros::Duration(3.0); // 3 seconds
  }
  else
  {
    max_pointcloud_wait_time_=ros::Duration(max_wait_time); // [s]
  }
  double max_octomap_wait_time;
  if( !ros::param::get("/"+interface_namespace+"/initialization/stereo_camera/max_octomap_wait_time",max_octomap_wait_time) )
  {
    ROS_WARN_STREAM("StereoCameraDataRetriever:: No max_octomap_wait_time was found on parameter server ('"<<"/"+interface_namespace+"/initialization/stereo_camera/max_octomap_wait_time"<<"'), the default (10s) will be used.");
    max_octomap_wait_time_=ros::Duration(10.0); // 3 seconds
  }
  else
  {
    max_octomap_wait_time_=ros::Duration(max_octomap_wait_time); // [s]
  }
  if( !ros::param::get("/"+interface_namespace+"/initialization/stereo_camera/wait_for_octomap",wait_for_octomap_) )
  {
    ROS_WARN_STREAM("StereoCameraDataRetriever:: 'wait_for_octomap' wasn't specified - Will wait for octomap (default:true).");
  }
  
  octomap_has_published_=false;
    
  octomap_topic_subscriber_ = nh_.subscribe( octomap_topic,1, &dense_reconstruction::StereoCameraDataRetriever::octomapCallback, this );
  pcl_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>( pcl_out_topic, 1 );
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
  // subscribe to stereo_image_proc node
  pcl_subscriber_ = nh_.subscribe( pcl_in_topic_,1, &dense_reconstruction::StereoCameraDataRetriever::pclCallback, this );
  
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
  
  if( !wait_for_octomap_ )
  {
      return RobotPlanningInterface::RECEIVED;
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

void StereoCameraDataRetriever::pclCallback( const sensor_msgs::PointCloud2ConstPtr& _msg )
{
  if( republish_ )
  {
    pcl_publisher_.publish(_msg);
    republish_ = false;
    
    // dump data to file
    std::stringstream name;
    name<<"/home/stefan/bunny_set_"<<dataCount_;
    ++dataCount_;
    
    rosbag::Bag bag;
    bag.open(name.str(), rosbag::bagmode::Write);
    bag.write("pcl", ros::Time::now(), _msg );
    
    // don't listen anymore, only one reception is necessary
    pcl_subscriber_.shutdown();
  }
}

void StereoCameraDataRetriever::octomapCallback( const octomap_msgs::OctomapConstPtr& _msg )
{
  octomap_has_published_ = true;
}

}