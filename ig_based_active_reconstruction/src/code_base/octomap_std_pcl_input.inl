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
  , max_sensor_range_m(-1)
  {
    
  }
  
  TEMPT
  CSCOPE::StdPclInput( Config config )
  : link_()
  , config_(config)
  , occlusion_calculator_(nullptr)
  {
    
  }
  
  TEMPT
  void CSCOPE::push( const Eigen::Transform<double,3,Eigen::Affine>& sensor_to_world, POINTCLOUD_TYPE& pc )
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
    
    // insert points into octree through raycasting
    Eigen::Vector3d sensor_position = sensor_to_world.translation();
    
    using ::octomap::point3d;
    using ::octomap::KeySet;
    using ::octomap::KeyRay;
    using ::octomap::OcTreeKey;
    
    point3d sensor_origin(sensor_position(0),sensor_position(1),sensor_position(2));
    
    
    // build sets of free and occupied voxels
    KeySet free_cells, occupied_cells;
    KeyRay key_ray_temp;
    
    typename POINTCLOUD_TYPE::const_iterator it, end;
    for(it = pc_cpy->begin(), end = pc_cpy->end(); it != end; ++it)
    {
      point3d point(it->x, it->y, it->z);
      // maxrange check
      point3d curr_ray = point - sensor_origin;
      
      
      if ((config_.max_sensor_range_m< 0.0) || (curr_ray.norm() <= (config_.max_sensor_range_m+0.000001)) )
      {
	// free cells
	if(link_.octree->computeRayKeys(sensor_origin, point, key_ray_temp))
	{
	  free_cells.insert(key_ray_temp.begin(), key_ray_temp.end());
	}
	// occupied endpoint
	OcTreeKey key;
	if(link_.octree->coordToKeyChecked(point, key))
	{
	  occupied_cells.insert(key);
	}
      }
      else
      {
	// ray longer than max range
	point3d new_end = sensor_origin + curr_ray.normalized() * config_.max_sensor_range_m;
	if (link_.octree->computeRayKeys(sensor_origin, new_end, key_ray_temp))
	{
	  free_cells.insert(key_ray_temp.begin(), key_ray_temp.end());
	}
      }
    }
    
    // update occupancy likelihoods
    
    // mark free cells only if not seen occupied in this cloud - attention: voxels may already exist even though no actual measurement has yet been received at their position (e.g. if their occlusion distance was calculated) - need to check hasMeasurement()!
    for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it)
    {
      if( occupied_cells.find(*it) == occupied_cells.end() )
      {
	typename TREE_TYPE::NodeType* voxel = link_.octree->search(*it);
	
	if( voxel==nullptr )
	{
	  voxel = link_.octree->updateNode(*it, false);
	  voxel->updateHasMeasurement(true);
	}
	else
	{
	  if( !voxel->hasMeasurement() )
	  {
	    float logOddsFirstMiss = ::octomap::logodds( link_.octree->config().miss_probability );
	    voxel->setLogOdds(logOddsFirstMiss);
	    voxel->updateHasMeasurement(true);
	  }
	  else
	  {
	    link_.octree->updateNode(*it, false);
	  }
	}
      }
    }
    
    // now mark all occupied cells:
    for (KeySet::iterator it = occupied_cells.begin(), end=free_cells.end(); it!= end; ++it)
    {
      typename TREE_TYPE::NodeType* voxel = link_.octree->search(*it);
      
      if( voxel==nullptr )
      {
	voxel = link_.octree->updateNode(*it, true);
	voxel->updateHasMeasurement(true);
      }
      else
      {
	if( !voxel->hasMeasurement() )
	{
	  float logOddsFirstHit = ::octomap::logodds( link_.octree->config().hit_probability );
	  voxel->setLogOdds(logOddsFirstHit);
	  voxel->updateHasMeasurement(true);
	}
	else
	{
	  link_.octree->updateNode(*it, true);
	}
      }
    }
  }
  
  TEMPT
  void CSCOPE::setLink( typename WorldRepresentation<TREE_TYPE>::Link& link )
  {
    link_.octree = link.octree;
  }
  
  TEMPT
  void CSCOPE::setOctree( std::shared_ptr<TREE_TYPE> octree )
  {
    link_.octree = octree;
  }
  
  TEMPT
  template< template<typename,typename> class OCCLUSION_CALC_TYPE, class ... Types >
  void CSCOPE::setOcclusionCalculator( Types ... args )
  {
    occlusion_calculator_ = std::make_shared< OCCLUSION_CALC_TYPE<TREE_TYPE,POINTCLOUD_TYPE> >( args... );
    occlusion_calculator_->setLink(link_);
  }
}

}

}