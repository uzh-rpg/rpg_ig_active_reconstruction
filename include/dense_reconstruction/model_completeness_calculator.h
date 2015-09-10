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

#pragma once

#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>

#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/Octomap.h>

namespace dense_reconstruction
{
  
  // Class that calculates how much of a given model is covered by pointclouds...
  class ModelCompletenessCalculator
  {
  public:
    ModelCompletenessCalculator( std::string name_, std::string path_ )
    : name_(name_), path_(path_){};
    void setInputBag( std::string path );
    void setGroundTruth( std::string path );
    void calculateCompleteness( std::vector<double>& completeness );
    void setRegisterDistance( double distance );
    
    double calculateCompleteness( pcl::PointCloud<pcl::PointXYZ>& toCompare );
    
  private:
    std::string name_;
    std::string path_;
    
    rosbag::Bag bag_;
    double registerDistance_;
    
    std::vector<double> completeness;
    
    ros::Publisher groundTruthPbl_;
    std::vector<sensor_msgs::PointCloud2::Ptr> bagContent_;
    pcl::PointCloud<pcl::PointXYZ> groundTruth_;
  };
  
}