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

#include <pcl/common/projection_matrix.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ig_active_reconstruction_octomap/octomap_world_representation.hpp"
#include "ig_active_reconstruction_octomap/octomap_occlusion_calculator.hpp"

namespace ig_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  /*! Base class for pointcloud type input to octomap. Provides setOcclusionCalculator() functionality. Inherits setLink() property from LinkedObject class.
   */
  template<class TREE_TYPE, class POINTCLOUD_TYPE>
  class PclInput: public WorldRepresentation<TREE_TYPE>::LinkedObject
  {
  public:
    virtual ~PclInput(){};
    
    /*! Sets the octree in which occlusions will be marked.
     */
    virtual void setOctree( std::shared_ptr<TREE_TYPE> octree );
    
    /*! Inserts a new pointcloud. If an occlusion calculator was set, it is called at the end.
     * 
     * @param sensor_to_world Transform from sensor to world coordinates.
     * @param pcl The pointcloud that is to be inserted, in sensor coordinates. Note that the function will operate directly on it
     */
    virtual void push( const Eigen::Transform<double,3,Eigen::Affine>& sensor_to_world, POINTCLOUD_TYPE& pcl )=0;
    
    /*! Adds an occlusion calculator that will be called at the end of pointcloud insertions. 
     * It is expected to derive from OcclusionCalculator and to take two template arguments: TREE_TYPE and POINTCLOUD_TYPE.
     * 
     * Usage of the function is similar to std::make_shared, but you can omit the two template arguments.
     * E.g. setOcclusionCalculator<WhateverOcclusionCalculator>(0.1,"test",4) will internally create an object
     * by calling std::make_shared<WhateverOcclusionCalculator<TREE_TYPE,POINTCLOUD_TYPE> >(0.1,"test",4).
     * The template arguments of the occlusion calculator can thus be omitted and are derived directly from
     * the WorldRepresentation object on which you are using it.
     * 
     * @param args Whichever arguments the occlusion calculator expects. (variadic template)
     */
    template< template<typename,typename> class OCCLUSION_CALC_TYPE, class ... Types >
    void setOcclusionCalculator( Types ... args );
    
  protected:
    std::shared_ptr< OcclusionCalculator<TREE_TYPE,POINTCLOUD_TYPE> > occlusion_calculator_; //! Calculates occlusions
  };
}

}

}

#include "../src/code_base/octomap_pcl_input.inl"