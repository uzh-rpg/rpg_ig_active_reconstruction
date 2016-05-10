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

#include "ig_active_reconstruction_octomap/octomap_information_gain.hpp"

namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
namespace octomap
{  
  /*! Templated class that implements the area factor presented by Vasquez-Gomez et al. in their paper "Volumetric next best view planning for 3d object reconstruction with positioning error, Journal of Advanced Robotic Systems, vol.11, 2014"
   */
  template<class TREE_TYPE>
  class VasquezGomezAreaFactorIg: public InformationGain<TREE_TYPE>
  {
  public:
    typedef typename InformationGain<TREE_TYPE>::GainType GainType;
    typedef typename InformationGain<TREE_TYPE>::Utils::Config Config;
    
    class Utils: public InformationGain<TREE_TYPE>::Utils
    {
    public:
      struct LocalConfig
      {
      public:
	LocalConfig();
	
      public:
	double p_occupied_des_; //! Desired occupancy percentage. Default: 0.2
	double p_occplane_des_; //! Desired occplane percentage. Default: 0.8
      } vasquez_config;
      
    public:
      Utils( Config a_config = Config() ):InformationGain<TREE_TYPE>::Utils(a_config){};
    };
    
    
  public:
    
    /*! Constructor
     */
    VasquezGomezAreaFactorIg( Config config = Config() );
    
    /*! Returns the name of the method.
     */
    virtual std::string type();
    
    /*! Returns the information gain calculated for all data added so far (For all rays).
     */
    virtual GainType getInformation();
    
    /*! Clears all ray-specific data, next data will be considered part of new ray.
     */
    virtual void makeReadyForNewRay();
    
    /*! Tells the metric that the current computation has ended and any next call will refer to a new
     * one - resets all internal data, but not any "external" configuration.
     */
    virtual void reset();
    
    /*! Includes a measurement on the ray.
     * @param node Octomap node traversed by the ray.
     */
    virtual void includeRayMeasurement( typename TREE_TYPE::NodeType* node );
    
    /*! Includes the endpoint of a ray.
     * @param node Octomap node at end of ray.
     */
    virtual void includeEndPointMeasurement( typename TREE_TYPE::NodeType* node );
    
    /*! Informs the metric that a complete ray was cast through empty space without
     * retrieving any measurements.
     */
    virtual void informAboutVoidRay();
    
    /*! Returns the number of processed voxels
     */
    virtual uint64_t voxelCount();
    
  protected:
    /*! Helper function
     * @param node Octomap node traversed by the ray.
     */
    virtual bool includeMeasurement( typename TREE_TYPE::NodeType* node );
    
    /*! Helper function to speed up calculations: Precomputes needed coefficients for a given alpha.
     */
    void setCoefficients( double alpha, double& a_f1, double& b_f1, double& a_f2, double& b_f2, double& c_f2, double& d_f2 );
    
    /*! Area factor function as defined in paper with precalculated coefficients for occupied voxel percentage.
     * @return The area factor.
     */
    double areaFactorOccupied( double percentage );
    
    /*! Area factor function as defined in paper with precalculated coefficients for occplane voxel percentage.
     * @return The area factor.
     */
    double areaFactorOccplane( double percentage );
    
  private:
    Utils utils_; //! Providing configuration and often used tools.
    
    uint64_t occupied_count_; //! Occupied voxels passed.
    uint64_t occplane_count_; //! Occplane voxels count.
    uint64_t unobserved_count_; //! Unobserved voxels count.
    
    uint64_t voxel_count_; //! Voxels integrated for the information gain calculation.
    
    bool no_known_voxel_so_far_;
    bool previous_voxel_free_;
    bool ray_is_already_registered_; //! To ensure that every ray is only registered once.
    
    // Precomputed coefficients...
    // occupied...
    double a_f1_occu_, b_f1_occu_;
    double a_f2_occu_, b_f2_occu_, c_f2_occu_, d_f2_occu_;
    // occplane...
    double a_f1_occp_, b_f1_occp_;
    double a_f2_occp_, b_f2_occp_, c_f2_occp_, d_f2_occp_;
    
  };
}

}

}

#include "../src/code_base/ig/vasquez_gomez_area_factor.inl"