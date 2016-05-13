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

#define TEMPT template<class TREE_TYPE>
#define CSCOPE VasquezGomezAreaFactorIg<TREE_TYPE>


namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
namespace octomap
{  
  TEMPT
  CSCOPE::Utils::LocalConfig::LocalConfig()
  : p_occupied_des_(0.2)
  , p_occplane_des_(0.8)
  {
    
  }
  
  TEMPT
  CSCOPE::VasquezGomezAreaFactorIg( Config utils )
  : utils_(utils)
  , occupied_count_(0)
  , occplane_count_(0)
  , unobserved_count_(0)
  , voxel_count_(0)
  , no_known_voxel_so_far_(true)
  , previous_voxel_free_(true)
  , ray_is_already_registered_(false)
  {
    setCoefficients(utils_.vasquez_config.p_occupied_des_,
                    a_f1_occu_,
                    b_f1_occu_,
                    a_f2_occu_,
                    b_f2_occu_,
                    c_f2_occu_,
                    d_f2_occu_ );
    setCoefficients(utils_.vasquez_config.p_occplane_des_,
                    a_f1_occp_,
                    b_f1_occp_,
                    a_f2_occp_,
                    b_f2_occp_,
                    c_f2_occp_,
                    d_f2_occp_ );
  }
  
  TEMPT
  std::string CSCOPE::type()
  {
    return "VasquezGomezAreaFactorIg";
  }
  
  TEMPT
  typename CSCOPE::GainType CSCOPE::getInformation()
  {
    // using formulation by Vasquez-Gomez et al. to calculate information
    double voxel_sum = occupied_count_ + occplane_count_ + unobserved_count_;
    
    if( voxel_sum==0 )
        return 0;
    
    double occupied_vox_perc = occupied_count_ / voxel_sum;
    double occplane_vox_perc = occplane_count_ / voxel_sum;
    
    return areaFactorOccupied(occupied_vox_perc) + areaFactorOccplane(occplane_vox_perc);
  }
  
  TEMPT
  void CSCOPE::makeReadyForNewRay()
  {
    previous_voxel_free_ = true;
    ray_is_already_registered_ = false;
  }
  
  TEMPT
  void CSCOPE::reset()
  {
    voxel_count_ = 0;
    no_known_voxel_so_far_ = true;
    previous_voxel_free_ = true;
  }
  
  TEMPT
  void CSCOPE::includeRayMeasurement( typename TREE_TYPE::NodeType* node )
  {
    includeMeasurement(node);
  }
  
  TEMPT
  void CSCOPE::includeEndPointMeasurement( typename TREE_TYPE::NodeType* node )
  {
    if( !includeMeasurement(node) ) // if ray wasn't registered to the last voxel on it, it's an unmarked one
    {
      unobserved_count_+=1;
    }
  }
  
  TEMPT
  void CSCOPE::informAboutVoidRay()
  {
    // void ray corresponds to an unmarked one...
    unobserved_count_+=1;
  }
  
  TEMPT
  uint64_t CSCOPE::voxelCount()
  {
    return voxel_count_;
  }
  
  TEMPT
  bool CSCOPE::includeMeasurement( typename TREE_TYPE::NodeType* node )
  {
    if(ray_is_already_registered_) // register each ray only once
    {
        return true;
    }
    
    double occ;
    if( node==NULL )
    {
        occ=utils_.config.p_unknown_prior; // default for unknown
        if( no_known_voxel_so_far_ )
        {
            previous_voxel_free_ = true; // treat unknown at beginning of ray, starting from sensor position as p_free
        }
        else
        {
            previous_voxel_free_ = false; // treat unknown at beginning of ray, starting from sensor position as p_free
        }

        return false;
    }
    else
    {
        occ = utils_.pOccupancy(node);
    }
    
    if( !node->hasMeasurement() && node->occDist()!=-1 )
    {
        ++occplane_count_;
        ray_is_already_registered_ = true;
        previous_voxel_free_ = false;
        return true;
    }
    else
    {
        no_known_voxel_so_far_=false;
    }
    
    if( utils_.isOccupied(occ) ) // voxel is occupied
    {
        if( previous_voxel_free_ ) // occupied voxels should only be registered as occupied if they are actually expected to be visible (and not hit from the backside...)
        {
            ++occupied_count_;
        }
        previous_voxel_free_ = false;
        ray_is_already_registered_ = true;
        return true;
    }
    else if( utils_.isUnknown(occ) ) // voxel is unknown
    {
        previous_voxel_free_ = false;
        
    }
    else
    {
        previous_voxel_free_ = true;
        // void, empty voxels are not of interest
    }
    return false;
  }
  
  TEMPT
  void CSCOPE::setCoefficients( double alpha, double& a_f1, double& b_f1, double& a_f2, double& b_f2, double& c_f2, double& d_f2 )
  {
    double alpha2 = alpha*alpha;
    double alpha3 = alpha*alpha2;
    double a_m1 = alpha-1;
    double a_m1_3 = a_m1*a_m1*a_m1;
    
    a_f1 = -(2/alpha3);
    b_f1 = (3/alpha2);
    a_f2 = -(2/a_m1_3);
    b_f2 = ((3*alpha+3)/a_m1_3);
    c_f2 = -(6*alpha/a_m1_3);
    d_f2 = ((3*alpha-1)/a_m1_3);
  }
  
  TEMPT
  double CSCOPE::areaFactorOccupied( double percentage )
  {
    double x2 = percentage*percentage;
    double x3 = percentage*x2;
    
    if( percentage<=utils_.vasquez_config.p_occupied_des_ )
    {
	return a_f1_occu_*x3 + b_f1_occu_*x2;
    }
    else
    {
	return a_f2_occu_*x3 + b_f2_occu_*x2 + c_f2_occu_*percentage + d_f2_occu_;
    }
  }
  
  TEMPT
  double CSCOPE::areaFactorOccplane( double percentage )
  {
    double& x = percentage;
    double x2 = x*x;
    double x3 = x*x2;
    
    if( x<=utils_.vasquez_config.p_occplane_des_ )
    {
      return a_f1_occp_*x3 + b_f1_occp_*x2;
    }
    else
    {
      return a_f2_occp_*x3 + b_f2_occp_*x2 + c_f2_occp_*x + d_f2_occp_;
    }
  }
  
}

}

}

#undef CSCOPE
#undef TEMPT