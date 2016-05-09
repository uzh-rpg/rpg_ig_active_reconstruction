/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of ig_active_reconstruction, a ROS package for...well,

ig_active_reconstruction is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
ig_active_reconstruction is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with ig_active_reconstruction. If not, see <http://www.gnu.org/licenses/>.
*/

#define TEMPT template<class TREE_TYPE, class POINTCLOUD_TYPE>
#define CSCOPE RayOcclusionCalculator<TREE_TYPE,POINTCLOUD_TYPE>

#include <octomap/octomap.h>

namespace ig_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  TEMPT
  CSCOPE::RayOcclusionCalculator( Options options )
  : occlusion_update_dist_m_(options.occlusion_update_dist_m)
  {
    
  }
  
  TEMPT
  void CSCOPE::insert( const Eigen::Vector3d& origin, const POINTCLOUD_TYPE& pcl, std::vector<int>& valid_indices  )
  {
    if( this->link_.octree==NULL )
      return;
    
    using ::octomap::point3d;
    using ::octomap::KeyRay;
    
    point3d sensor_origin(origin(0),origin(1),origin(2));
    KeyRay ray;
    
    //typename POINTCLOUD_TYPE::const_iterator it, end;    
    //for(it = pcl.begin(), end = pcl.end(); it != end; ++it)
    for( size_t i = 0; i<valid_indices.size(); ++i )
    {
      //point3d point(it->x, it->y, it->z);
      point3d point(pcl.points[valid_indices[i]].x, pcl.points[valid_indices[i]].y, pcl.points[valid_indices[i]].z);
      point3d curr_ray = point - sensor_origin;
      
      point3d new_end = point + curr_ray.normalized() * occlusion_update_dist_m_;
      if (this->link_.octree->computeRayKeys(point, new_end, ray))
      {
	  KeyRay::iterator occ = ray.begin(); // first point is the occupied one - skip it!
	  KeyRay::iterator end = ray.end();
	  
	  if(occ!=end)
	  {
	      for( unsigned int i=0; occ!=end; ++i, ++occ )
	      {
		  typename TREE_TYPE::NodeType* voxel = this->link_.octree->search(*occ);
				  
		  if( voxel!=NULL )
		  {
		      if( !voxel->hasMeasurement() )
		      {
			  voxel->updateOccDist( i );
		      }
		      else
		      {
			  break; // don't trace through known occupied/empty voxels
		      }
		  }
		  else
		  {
		      voxel = this->link_.octree->updateNode(*occ, false, 0.1); //random distance
		      // the occupancy probability will be ignored during an actual update with the following call:
		      voxel->updateHasMeasurement(false);
		      voxel->updateOccDist( i );
		  }
	      }
	  }
      }
    }
  }
  
  TEMPT
  void CSCOPE::setOctree( boost::shared_ptr<TREE_TYPE> octree )
  {
    this->link_.octree = octree;
  }
}

}

}

#undef CSCOPE
#undef TEMPT