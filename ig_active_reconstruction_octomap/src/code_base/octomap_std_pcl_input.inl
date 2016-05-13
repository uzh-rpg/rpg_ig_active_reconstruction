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
#define CSCOPE StdPclInput<TREE_TYPE, POINTCLOUD_TYPE>

#include <limits>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter_indices.h>

namespace ig_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  TEMPT
  CSCOPE::Config::Config()
  : use_bounding_box(false)
  , bounding_box_min_point_m( -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max() )
  , bounding_box_max_point_m( std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max() )
  , max_sensor_range_m(-1)
  {
    
  }
  
  TEMPT
  CSCOPE::StdPclInput( Config config )
  : config_(config)
  {
    
  }
  
  TEMPT
  void CSCOPE::push( const Eigen::Transform<double,3,Eigen::Affine>& sensor_to_world, POINTCLOUD_TYPE& pc )
  {
    pcl::transformPointCloud(pc, pc, sensor_to_world);
    
    typename POINTCLOUD_TYPE::Ptr pc_cpy = pc.makeShared();
    
    std::vector<int> valid_indices;
    
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
    
    pcl::removeNaNFromPointCloud(*pc_cpy,valid_indices);
    
    std::cout<<"Inserting "<<pc_cpy->points.size()<<" valid points.";
    
    
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
    //for(it = pc_cpy->begin(), end = pc_cpy->end(); it != end; ++it)
    for( size_t i = 0; i<valid_indices.size(); ++i )
    {
      //point3d point(it->x, it->y, it->z);
      point3d point(pc_cpy->points[valid_indices[i]].x, pc_cpy->points[valid_indices[i]].y, pc_cpy->points[valid_indices[i]].z);
      // maxrange check
      point3d curr_ray = point - sensor_origin;
      
      if(i%100==0)
	std::cout<<"\nBuilding iterator set: "<<i<<"/"<<valid_indices.size();
      
      if ((config_.max_sensor_range_m< 0.0) || (curr_ray.norm() <= (config_.max_sensor_range_m+0.000001)) )
      {
	// free cells
	if(this->link_.octree->computeRayKeys(sensor_origin, point, key_ray_temp))
	{
	  free_cells.insert(key_ray_temp.begin(), key_ray_temp.end());
	}
	// occupied endpoint
	OcTreeKey key;
	if(this->link_.octree->coordToKeyChecked(point, key))
	{
	  occupied_cells.insert(key);
	}
      }
      else
      {
	// ray longer than max range
	point3d new_end = sensor_origin + curr_ray.normalized() * config_.max_sensor_range_m;
	if (this->link_.octree->computeRayKeys(sensor_origin, new_end, key_ray_temp))
	{
	  free_cells.insert(key_ray_temp.begin(), key_ray_temp.end());
	}
      }
    }
    
    // update occupancy likelihoods
    
    // mark free cells only if not seen occupied in this cloud - attention: voxels may already exist even though no actual measurement has yet been received at their position (e.g. if their occlusion distance was calculated) - need to check hasMeasurement()!
    size_t count = 0;
    for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it)
    {
      if( count++%1000==0)
	std::cout<<"\nInserting free: "<<count<<"/"<<free_cells.size();
      
      if( occupied_cells.find(*it) == occupied_cells.end() )
      {
	typename TREE_TYPE::NodeType* voxel = this->link_.octree->search(*it);
	
	if( voxel==NULL )
	{
	  voxel = this->link_.octree->updateNode(*it, false);
	  voxel->updateHasMeasurement(true);
	}
	else
	{
	  if( !voxel->hasMeasurement() )
	  {
	    float logOddsFirstMiss = ::octomap::logodds( this->link_.octree->config().miss_probability );
	    voxel->setLogOdds(logOddsFirstMiss);
	    voxel->updateHasMeasurement(true);
	  }
	  else
	  {
	    this->link_.octree->updateNode(*it, false);
	  }
	}
      }
    }
    
    count = 0;
    // now mark all occupied cells:
    for (KeySet::iterator it = occupied_cells.begin(), end=free_cells.end(); it!= end; ++it)
    {
      if( count++%100==0)
	std::cout<<"\nInserting occupied: "<<count<<"/"<<occupied_cells.size();
      
      typename TREE_TYPE::NodeType* voxel = this->link_.octree->search(*it);
      
      if( voxel==NULL )
      {
	voxel = this->link_.octree->updateNode(*it, true);
	voxel->updateHasMeasurement(true);
      }
      else
      {
	if( !voxel->hasMeasurement() )
	{
	  float logOddsFirstHit = ::octomap::logodds( this->link_.octree->config().hit_probability );
	  voxel->setLogOdds(logOddsFirstHit);
	  voxel->updateHasMeasurement(true);
	}
	else
	{
	  this->link_.octree->updateNode(*it, true);
	}
      }
    }
    if( this->occlusion_calculator_!=NULL )
    {
      std::cout<<"\nCalling occlusion calculator";
      this->occlusion_calculator_->insert(sensor_position,*pc_cpy,valid_indices);
    }
    
    std::cout<<"\nFinsihed calculations";
  }
  
  
}

}

}

#undef CSCOPE
#undef TEMPT