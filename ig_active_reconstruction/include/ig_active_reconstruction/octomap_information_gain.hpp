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

#include "ig_active_reconstruction/factory.hpp"

namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
namespace octomap
{  
  /*! Defines the interface for information gain calculators as well as providing a factory.
   * 
   * Information gain classes are called while casting rays from a sensor position through the octomap.
   * Before a new ray is cast, makeReadyForNewRay() is called, then includeRayMeasurement(...) is called
   * on every voxel the ray traversed but the last one, for which includeEndPointMeasurement(...) is called.
   * If a ray was cast through empty (unknown, uninitialized) space solely, informAboutVoidRay() is called.
   * The calculated information gain is retrieved after the last ray was cast through getInformation().
   */
  template<class TREE_TYPE>
  class InformationGain
  {
  public:
    typedef double GainType;
    
    /*! Factory class where information gain types must be registered and can afterwards
     * be retrieved through their unique id or name.
     */
    typedef multikit::Factory<InformationGain> Factory;
    
  public:
    /*! Returns the factory object.
     */
    static Factory& factory();
    
  public:
    /*! Links the octree on which information gains will be calculated.
     * @param octree Pointer to the octree.
     */
    virtual void setOctree( std::shared_ptr<TREE_TYPE> octree )=0;
    
    /*! Returns the name of the method.
     */
    virtual std::string type()=0;
    
    /*! Returns the information gain calculated for all data added so far (For all rays).
     */
    virtual GainType getInformation()=0;
    
    /*! Clears all ray-specific data, next data will be considered part of new ray.
     */
    virtual void makeReadyForNewRay()=0;
    
    /*! Includes a measurement on the ray.
     * @param node Octomap node traversed by the ray.
     */
    virtual void includeRayMeasurement( typename TREE_TYPE::NodeType* node )=0;
    
    /*! Includes the endpoint of a ray.
     * @param node Octomap node at end of ray.
     */
    virtual void includeEndPointMeasurement( typename TREE_TYPE::NodeType* node )=0;
    
    /*! Informs the metric that a complete ray was cast through empty space without
     * retrieving any measurements.
     */
    virtual void informAboutVoidRay()=0;
    
    /*! Returns the number of traversed voxels
     */
    virtual unsigned int voxelCount()=0;
    
    
  };
  
}

}

}

#include "../src/code_base/octomap_information_gain.inl"