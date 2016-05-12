/* Copyright (c) 2016, Stefan Isler, islerstefan@bluewin.ch
 * (ETH Zurich / Robotics and Perception Group, University of Zurich, Switzerland)
 *
 * This file is part of ig_active_reconstruction, software for information gain based, active reconstruction.
 *
 * ig_active_reconstruction is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * ig_active_reconstruction is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * Please refer to the GNU Lesser General Public License for details on the license,
 * on <http://www.gnu.org/licenses/>.
*/

#include "flying_gazebo_stereo_cam/pcl_rerouter.hpp"
#include "ig_active_reconstruction_msgs/PclInput.h"

namespace ros_tools
{
  
  PclRerouter::PclRerouter( ros::NodeHandle nh, std::string in_name, std::string out_name  )
  : nh_(nh)
  , forward_one_(false)
  , has_published_one_(false)
  , one_to_srv_(false)
  {
    pcl_subscriber_ = nh_.subscribe( in_name,1, &PclRerouter::pclCallback, this );
    pcl_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(out_name, 1);
    pcl_service_caller_ = nh_.serviceClient<ig_active_reconstruction_msgs::PclInput>(out_name);
  }
  
  bool PclRerouter::rerouteOneToTopic(ros::Duration max_wait_time)
  {
    has_published_one_ = false;
    forward_one_ = true;
    ros::spinOnce();
    
    ros::Time time_limit = ros::Time::now() + max_wait_time;
    ros::Duration sleep_time;
    sleep_time.fromNSec(max_wait_time.toNSec()/10);
    
    while(!has_published_one_ && ros::Time::now()<time_limit)
    {
      ros::spinOnce();
      sleep_time.sleep();
    }
    forward_one_ = false;
    
    return has_published_one_;
  }
  
  bool PclRerouter::rerouteOneToSrv()
  {
    has_published_one_ = false;
    one_to_srv_ = true;
    ros::spinOnce();
    
    while( one_to_srv_ && nh_.ok() )
    {
      ros::spinOnce();
      ros::Duration(0.01).sleep();
    }
    return service_response_;
  }
  
  void PclRerouter::pclCallback( const sensor_msgs::PointCloud2ConstPtr& msg )
  {
    if(forward_one_)
    {
      pcl_publisher_.publish(msg);
      has_published_one_ = true;
      forward_one_ = false;
    }
    else if(one_to_srv_)
    {
      ig_active_reconstruction_msgs::PclInput call;
      call.request.pointcloud = *msg;
      pcl_service_caller_.call(call);
      service_response_ = call.response.success;
      one_to_srv_ = false;
    }
    
    return;
  }
  
}