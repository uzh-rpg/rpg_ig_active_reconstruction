/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of ig_based_active_reconstruction, a ROS package for...well,

ig_based_active_reconstruction is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
ig_based_active_reconstruction is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with ig_based_active_reconstruction. If not, see <http://www.gnu.org/licenses/>.
*/

#define TEMPT template<class TREE_TYPE, class POINTCLOUD_TYPE>
#define CSCOPE StdPclInput<TREE_TYPE, POINTCLOUD_TYPE>

#include <limits>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

namespace ig_based_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  TEMPT
  CSCOPE::Config::Config()
  : use_bounding_box(false)
  , bounding_box_min_point_m( std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min() )
  , bounding_box_max_point_m( std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max() )
  {
    
  }
  
  TEMPT
  CSCOPE::StdPclInput( Config config )
  : octree_(nullptr)
  , config_(config)
  {
    
  }
  
  TEMPT
  void CSCOPE::push( const Eigen::Matrix4d& sensor_to_world, POINTCLOUD_TYPE& pc )
  {
    pcl::transformPointCloud(pc, pc, sensor_to_world);
    
    typename POINTCLOUD_TYPE::Ptr pc_cpy = pc.makeShared();
    
    // filter out everything not within bounding box, also removes NANS
    if( config_.use_bounding_box )
    {
      pcl::PassThrough<typename POINTCLOUD_TYPE::PointType> pass_z;
      pass_z.setFilterFieldName("z");
      pass_z.setFilterLimits(config_.bounding_box_min_point_m.z(), config_.bounding_box_max_point_m.z());
      
      pcl::PassThrough<typename POINTCLOUD_TYPE::PointType> pass_x;
      pass_x.setFilterFieldName("x");
      pass_x.setFilterLimits(config_.bounding_box_min_point_m.x(), config_.bounding_box_max_point_m.x());
      
      pcl::PassThrough<typename POINTCLOUD_TYPE::PointType> pass_y;
      pass_y.setFilterFieldName("y");
      pass_y.setFilterLimits(config_.bounding_box_min_point_m.y(), config_.bounding_box_max_point_m.y());
      
      pass_z.setInputCloud(pc_cpy);
      pass_z.filter(*pc_cpy);
      pass_x.setInputCloud(pc_cpy);
      pass_x.filter(*pc_cpy);
      pass_y.setInputCloud(pc_cpy);
      pass_y.filter(*pc_cpy);
    }
    
    // insert points into octree
    TODO // TODO
  }
  
  TEMPT
  void CSCOPE::setOctree( std::shared_ptr<TREE_TYPE> octree )
  {
    octree_ = octree;
  }
}

}

}