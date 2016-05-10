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
#define CSCOPE OcclusionAwareIg<TREE_TYPE>


namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
namespace octomap
{  
  TEMPT
  CSCOPE::OcclusionAwareIg( Config utils )
  : utils_(utils)
  , ig_(0)
  , p_vis_(1)
  , voxel_count_(0)
  {
    
  }
  
  TEMPT
  std::string CSCOPE::type()
  {
    return "OcclusionAwareIg";
  }
  
  TEMPT
  typename CSCOPE::GainType CSCOPE::getInformation()
  {
    return ig_;
  }
  
  TEMPT
  void CSCOPE::makeReadyForNewRay()
  {
    p_vis_ = 1.0;
  }
  
  TEMPT
  void CSCOPE::reset()
  {
    ig_ = 0;
    p_vis_ = 1;
    voxel_count_ = 0;
  }
  
  TEMPT
  void CSCOPE::includeRayMeasurement( typename TREE_TYPE::NodeType* node )
  {
    includeMeasurement(node);
  }
  
  TEMPT
  void CSCOPE::includeEndPointMeasurement( typename TREE_TYPE::NodeType* node )
  {
    includeMeasurement(node);
  }
  
  TEMPT
  void CSCOPE::informAboutVoidRay()
  {
    voxel_count_ += utils_.config.voxels_in_void_ray;
    // no approximation needed, can be exactly calculated using the geometric series formula
    ig_ += utils_.entropy(utils_.config.p_unknown_prior)/(1-utils_.config.p_unknown_prior); // information in void ray...
  }
  
  TEMPT
  uint64_t CSCOPE::voxelCount()
  {
    return voxel_count_;
  }
  
  TEMPT
  void CSCOPE::includeMeasurement( typename TREE_TYPE::NodeType* node )
  {
    ++voxel_count_;
    double p_occ = utils_.pOccupancy(node);
    double vox_ent = utils_.entropy(p_occ);
    ig_ += p_vis_*vox_ent;
    p_vis_ *= p_occ;
  }
  
}

}

}

#undef CSCOPE
#undef TEMPT