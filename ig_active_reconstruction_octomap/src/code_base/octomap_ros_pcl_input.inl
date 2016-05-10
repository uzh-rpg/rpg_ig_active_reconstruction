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

#define TEMPT template<class TREE_TYPE, class POINTCLOUD_TYPE>
#define CSCOPE RosPclInput<TREE_TYPE,POINTCLOUD_TYPE>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

namespace ig_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  TEMPT
  CSCOPE::RosPclInput( ros::NodeHandle nh, boost::shared_ptr< PclInput<TREE_TYPE,POINTCLOUD_TYPE> > pcl_input, std::string world_frame )
  : nh_(nh)
  , pcl_input_(pcl_input)
  , world_frame_name_(world_frame)
  , tf_listener_(ros::Duration(180))
  {
    pcl_subscriber_ = nh_.subscribe("pcl_input",10,&CSCOPE::insertCloudCallback,this);
    pcl_input_service_ = nh_.advertiseService("pcl_input", &CSCOPE::insertCloudService,this);
  }
  
  TEMPT
  void CSCOPE::addInputDoneSignalCall( boost::function<void()> signal_call )
  {
    signal_call_stack_.push_back(signal_call);
  }
  
  TEMPT
  void CSCOPE::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
  {
    ROS_INFO("Received new pointcloud. Inserting...");
    POINTCLOUD_TYPE pc;
    pcl::fromROSMsg(*cloud, pc);
    
    insertCloud(pc);
    ROS_INFO("Inserted new pointcloud");
  }
  
  TEMPT
  bool CSCOPE::insertCloudService( ig_active_reconstruction_msgs::PclInput::Request& req, ig_active_reconstruction_msgs::PclInput::Response& res)
  {
    ROS_INFO("Received new pointcloud. Inserting...");
    POINTCLOUD_TYPE pc;
    pcl::fromROSMsg(req.pointcloud, pc);
    
    insertCloud(pc);
    
    ROS_INFO("Inserted new pointcloud");
    res.success = true;
    return true;
  }
  
  TEMPT
  void CSCOPE::issueInputDoneSignals()
  {
    BOOST_FOREACH( boost::function<void()>& call, signal_call_stack_)
    {
      call();
    }
  }
  
  TEMPT
  void CSCOPE::insertCloud( POINTCLOUD_TYPE& pointcloud )
  {
    tf::StampedTransform sensor_to_world_tf;
    try
    {
      tf_listener_.lookupTransform(world_frame_name_, pointcloud.header.frame_id, ros::Time(0), sensor_to_world_tf);
    }
    catch(tf::TransformException& ex)
    {
      ROS_ERROR_STREAM( "RosPclInput<TREE_TYPE,POINTCLOUD_TYPE>::Transform error of sensor data: " << ex.what() << ", quitting callback.");
      return;
    }
    
    Eigen::Matrix4f sensor_to_world;
    pcl_ros::transformAsMatrix(sensor_to_world_tf, sensor_to_world);
    
    Eigen::Transform<double,3,Eigen::Affine> sensor_to_world_transform;
    sensor_to_world_transform = sensor_to_world.cast<double>();
    
    pcl_input_->push(sensor_to_world_transform,pointcloud);
    
    
    
    issueInputDoneSignals();
  }
  
}

}

}

#undef CSCOPE
#undef TEMPT