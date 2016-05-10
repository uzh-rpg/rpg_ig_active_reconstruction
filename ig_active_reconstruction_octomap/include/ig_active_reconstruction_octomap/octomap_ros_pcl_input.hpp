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

#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include "ig_active_reconstruction_msgs/PclInput.h"

#include "ig_active_reconstruction_octomap/octomap_pcl_input.hpp"

namespace ig_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  
  /*! Class subscribes to ROS pcl topic and feeds it to an octomap::PclInput
   * 
   * Subscribes to "pcl_input" on the passed ros node.
   * Advertices "pcl_input" as service
   */
  template<class TREE_TYPE, class POINTCLOUD_TYPE>
  class RosPclInput
  {
  public:
    /*! Constructor.
     * @param nh ros node handle under which topic and service will be advertised.
     * @param pcl_input PclInput object pointer to which pointclouds are forwarded.
     * @param world_frame Name of the world coordinate frame to which the incoming pointclouds will be transformed.
     */
    RosPclInput( ros::NodeHandle nh, boost::shared_ptr< PclInput<TREE_TYPE,POINTCLOUD_TYPE> > pcl_input, std::string world_frame );
    
    /*! Add a function that will be called after a new input was processed.
     * @param signal_call The function.
     */
    void addInputDoneSignalCall( boost::function<void()> signal_call );
    
  protected:
    /*! Pcl input topic listener.
     */
    void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    
    /*! Pcl input service.
     */
    bool insertCloudService( ig_active_reconstruction_msgs::PclInput::Request& req, ig_active_reconstruction_msgs::PclInput::Response& res);
    
    /*! Helper function calling the signal call stack.
     */
    void issueInputDoneSignals();
    
    /*! Inserts a cloud by reference and calls issueInputDoneSignals when done.
     */
    void insertCloud( POINTCLOUD_TYPE& pointcloud );
    
  private:
    ros::NodeHandle nh_;
    boost::shared_ptr< PclInput<TREE_TYPE,POINTCLOUD_TYPE> > pcl_input_;
    
    std::string world_frame_name_;
    
    std::vector< boost::function<void()> > signal_call_stack_;
    
    ros::Subscriber pcl_subscriber_;
    ros::ServiceServer pcl_input_service_;
    
    tf::TransformListener tf_listener_;
  };
}

}

}

#include "../src/code_base/octomap_ros_pcl_input.inl"