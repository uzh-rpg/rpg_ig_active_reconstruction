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

#include "ig_active_reconstruction_octomap/octomap_world_representation.hpp"

namespace ig_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  
  /*! This class provides a ROS interface to an octomap::WorldRepresentation, providing common
   * functionality like publishers and services.
   * Inputs (e.g. pcl) are not provided as these are supposed to be provided by dedicated classes in order
   * to allow a multiple sensor and multiple sensor modality approach.
   */
  template<class TREE_TYPE>
  class RosInterface: public WorldRepresentation<TREE_TYPE>::LinkedObject
  {
  public:
    typedef boost::shared_ptr< RosInterface<TREE_TYPE> > Ptr;
    typedef TREE_TYPE TreeType;
    
    struct Config
    {
      ros::NodeHandle nh;
      std::string world_frame_name;
    };
    
  public:
    RosInterface(Config config);
    
    /*! Publishes the voxel map as a visualization_msgs::MarkerArray.
     */
    void publishVoxelMap();
  protected:
    //virtual bool octomapBinarySrv(OctomapSrv::Request  &req, OctomapSrv::GetOctomap::Response &res);
    //virtual bool octomapFullSrv(OctomapSrv::Request  &req, OctomapSrv::GetOctomap::Response &res);
    //bool clearBBXSrv(BBXSrv::Request& req, BBXSrv::Response& resp);
    //bool resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
    
  private:
    ros::NodeHandle nh_;
    std::string world_frame_name_;
    ros::Publisher voxel_map_publisher_;
  };
  
}

}

}

#include "../src/code_base/octomap_ros_interface.inl"