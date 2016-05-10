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

#include "ig_active_reconstruction_octomap/octomap_world_representation.hpp"

#include <pcl/common/projection_matrix.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ig_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  /*! Abstract base class: Calculates occlusion distance updates.
   */
  template<class TREE_TYPE, class POINTCLOUD_TYPE>
  class OcclusionCalculator: public WorldRepresentation<TREE_TYPE>::LinkedObject
  {
  public:
    virtual ~OcclusionCalculator(){};
    
    /*! Calculates occlusion distances for the given input
     * and sets the respective values within the octree
     * @param origin Origin of the sensor, position from which pointcloud was obtained.
     * @param pcl The pointcloud
     * @param valid_indices Vector with the indices of all points in the pointcloud that should be considered
     */
    virtual void insert( const Eigen::Vector3d& origin, const POINTCLOUD_TYPE& pcl, std::vector<int>& valid_indices )=0;
    
    /*! Sets the octree in which occlusions will be marked.
     */
    virtual void setOctree( boost::shared_ptr<TREE_TYPE> octree )=0;
  };
  
}

}

}