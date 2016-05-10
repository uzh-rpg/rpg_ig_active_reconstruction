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

namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
namespace octomap
{  
  /*! Defines the interface for information gain calculators as well as providing a factory.
   * Additionally it features a Utils class which provides functionality like occupancy and entropy calculation.
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
    typedef boost::shared_ptr< InformationGain<TREE_TYPE> > Ptr;
        
    /*! Helper class for common calculation in information gain metrics.
     */
    class Utils
    {
    public:
      /*! Configuration of the Utils class, featuring default.
       */
      struct Config
      {
      public:
	Config();
	
      public:
	double p_unknown_prior;//! Prior occupancy likelihood for unknown voxels. Default: 0.5.
	double p_unknown_upper_bound; //! Upper bound for voxels to still be considered uncertain. Default: 0.8.
	double p_unknown_lower_bound; //! Lower bound for voxels to still be considered uncertain. Default: 0.2.
	unsigned int voxels_in_void_ray; //! How many voxels are considered to be part of a void ray. Default: 100.
      } config;
      
    public:
      Utils( Config a_config = Config() ):config(a_config){};
      
      /*! Returns the entropy for a voxel.
       */
      double entropy( typename TREE_TYPE::NodeType* voxel );
      
      /*! Calculates the entropy [nat] for a given likelihood.
       */
      double entropy( double likelihood );
      
      /*! Returns the occupancy probability of the passed voxel.
       */
      double pOccupancy( typename TREE_TYPE::NodeType* voxel );
      
      /*! Returns true if the likelihood lies within the unknown bounds.
       */
      bool isUnknown( typename TREE_TYPE::NodeType* voxel);
      
      /*! Returns true if the likelihood lies within the unknown bounds.
       */
      bool isUnknown( double likelihood);
      
      /*! Returns true if the voxel is considered occupied.
       */
      bool isOccupied( typename TREE_TYPE::NodeType* voxel);
      
      /*! Returns true if the likelihood lies above the occupied threshold.
       */
      bool isOccupied( double likelihood);
      
      /*! Returns true if the voxel is considered free.
       */
      bool isFree( typename TREE_TYPE::NodeType* voxel);
      
      /*! Returns true if the likelihood lies below the occupied threshold.
       */
      bool isFree( double likelihood);
    };
    
    typedef typename Utils::Config Config;
    
  public:
    
    /*! Returns the name of the method.
     */
    virtual std::string type()=0;
    
    /*! Returns the information gain calculated for all data added so far (For all rays).
     */
    virtual GainType getInformation()=0;
    
    /*! Clears all ray-specific data, next data will be considered part of new ray.
     */
    virtual void makeReadyForNewRay()=0;
    
    /*! Tells the metric that the current computation has ended and any next call will refer to a new
     * one - resets all internal data, but not any "external" configuration.
     */
    virtual void reset()=0;
    
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
    virtual uint64_t voxelCount()=0;
    
    
  };
  
}

}

}

#include "../src/code_base/octomap_information_gain.inl"