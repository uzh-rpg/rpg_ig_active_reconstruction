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

#pragma once

#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ig_based_active_reconstruction/octomap_occlusion_calculator.hpp"

namespace ig_based_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  
  template<class TREE_TYPE, class POINTCLOUD_TYPE = pcl::PointCloud<pcl::PointXYZ> >
  class WorldRepresentation
  {
  public:
    typedef POINTCLOUD_TYPE PclPointCloud;
    
  public:
    WorldRepresentation( typename TREE_TYPE::Config config );
    
    virtual ~WorldRepresentation();
    
    /*! Sets the occlusion calculator that should be used. It is expected to derive from OcclusionCalculator
     * and to take two template arguments: TREE_TYPE and POINTCLOUD_TYPE.
     * 
     * Usage of the function is similar to std::make_shared, but you can omit the two template arguments.
     * E.g. setOcclusionCalculator<WhateverOcclusionCalculator>(0.1,"test",4) will internally create an object
     * by calling std::make_shared<WhateverOcclusionCalculator<TREE_TYPE,PclPointCloud> >(0.1,"test",4).
     * The template arguments of the occlusion calculator can thus be omitted and are derived directly from
     * the WorldRepresentation object on which you are using it.
     * 
     * @param args Whichever arguments the occlusion calculator expects. (variadic template)
     */
    template< template<typename,typename> class OCCLUSION_CALC_TYPE, class ... Types >
    void setOcclusionCalculator( Types ... args );
    
    /*! Returns a pointer to an input object, setting its expected TREE_TYPE and POINTCLOUD_TYPE
     * template arguments automatically and calling setOctree() on it.
     * 
     * Usage is the same as for setOcclusionCalculator
     * 
     * @param args Whichever arguments the input object type expects. (variadic template)
     */
    template< template<typename,typename> class INPUT_OBJ_TYPE, class ... Types >
    std::shared_ptr< INPUT_OBJ_TYPE<TREE_TYPE,POINTCLOUD_TYPE> > getInputObj( Types ... args );
    
    /*! Calculates occlusions for the given input.
     * @param origin Origin of the sensor, position from which pointcloud was obtained.
     * @param pcl The pointcloud
     */
    void computeOcclusions( const Eigen::Vector3d& origin, const PclPointCloud& pcl );
    
    /*! Returns if occlusions have been calculated, i.e. if calculateOcclusions() has been called at least once.
     * If not, the class assumes that this is not the case.
     */
    bool knowsOcclusions();
    
  protected:
    std::shared_ptr<TREE_TYPE> octree_; //! Octomap tree instance.
    bool calculated_occlusions_; //! Whether occlusions have been calculated or not.
    
    std::shared_ptr< OcclusionCalculator<TREE_TYPE,PclPointCloud> > occlusion_calculator_; //! Calculates occlusions
  };
  
}

}

}

#include "../src/code_base/octomap_world_representation.inl"