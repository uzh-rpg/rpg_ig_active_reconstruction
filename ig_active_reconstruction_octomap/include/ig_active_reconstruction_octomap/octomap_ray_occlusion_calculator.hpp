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

#pragma once

#include "ig_active_reconstruction_octomap/octomap_occlusion_calculator.hpp"

namespace ig_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  /*! Calculates occlusion distances for a given distance along rays behind points.
   */
  template<class TREE_TYPE, class POINTCLOUD_TYPE>
  class RayOcclusionCalculator: public OcclusionCalculator<TREE_TYPE,POINTCLOUD_TYPE>
  {
  public:
    struct Options
    {
    public:
      Options( double occlusion_update_dist_m ): occlusion_update_dist_m(occlusion_update_dist_m){};
      
    public:
      double occlusion_update_dist_m; //! Max distance behind points along ray for which an occlusion will be calculated [m].
    };
    
  public:
    /*! Constructor
     * @param occlusion_update_dist_m 
     */
    RayOcclusionCalculator( Options options );
    
    /*! Calculates occlusion distances for the given input
     * and sets the respective values within the octree
     * @param origin Origin of the sensor, position from which pointcloud was obtained.
     * @param pcl The pointcloud
     */
    virtual void insert( const Eigen::Vector3d& origin, const POINTCLOUD_TYPE& pcl );
    
    /*! Sets the octree in which occlusions will be marked.
     */
    virtual void setOctree( boost::shared_ptr<TREE_TYPE> octree );
    
  protected:
    double occlusion_update_dist_m_; //! Max distance behind points along ray for which an occlusion will be calculated [m].
  };
  
}

}

}

#include "../src/code_base/octomap_ray_occlusion_calculator.inl"