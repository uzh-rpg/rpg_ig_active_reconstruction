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
#define CSCOPE RearSideEntropyIg<TREE_TYPE>


namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
namespace octomap
{  
  TEMPT
  CSCOPE::RearSideEntropyIg( Config utils )
  : utils_(utils)
  , ig_(0)
  , voxel_count_(0)
  , current_ray_ig_(0)
  , current_ray_voxel_count_(0)
  , p_vis_(1)
  , previous_voxel_unknown_(false)
  {
    
  }
  
  TEMPT
  std::string CSCOPE::type()
  {
    return "RearSideEntropyIg";
  }
  
  TEMPT
  typename CSCOPE::GainType CSCOPE::getInformation()
  {
    return ig_;
  }
  
  TEMPT
  void CSCOPE::makeReadyForNewRay()
  {
    current_ray_ig_ = 0;
    p_vis_ = 1;
    previous_voxel_unknown_ = false;
    current_ray_voxel_count_ = 0;
  }
  
  TEMPT
  void CSCOPE::reset()
  {
    ig_ = 0;
    voxel_count_ = 0;
    current_ray_ig_ = 0;
    p_vis_ = 1;
    previous_voxel_unknown_ = false;
    current_ray_voxel_count_ = 0;
  }
  
  TEMPT
  void CSCOPE::includeRayMeasurement( typename TREE_TYPE::NodeType* node )
  {
    includeMeasurement(node);
  }
  
  TEMPT
  void CSCOPE::includeEndPointMeasurement( typename TREE_TYPE::NodeType* node )
  {
    if( previous_voxel_unknown_ )
    {
      if( node==NULL || !node->hasMeasurement() ) // end point in free area...
	return;
    }
    double p_occ = utils_.pOccupancy(node);
    
    if( !utils_.isOccupied(p_occ) )
      return;
    
    double vox_ent = utils_.entropy(p_occ);
    current_ray_ig_ += p_vis_*vox_ent;
    current_ray_voxel_count_+=1;
    
    ig_+=current_ray_ig_;
    voxel_count_+=current_ray_ig_;
  }
  
  TEMPT
  void CSCOPE::informAboutVoidRay()
  {
    // didn't hit anything -> no rear side voxels...
    previous_voxel_unknown_ = false;
  }
  
  TEMPT
  uint64_t CSCOPE::voxelCount()
  {
    return voxel_count_;
  }
  
  TEMPT
  void CSCOPE::includeMeasurement( typename TREE_TYPE::NodeType* node )
  {
    double p_occ = utils_.pOccupancy(node);
    
    if( utils_.isUnknown(p_occ) )
    {
      previous_voxel_unknown_ = true;
      double vox_ent = utils_.entropy(p_occ);
      current_ray_ig_ += p_vis_*vox_ent;
      current_ray_voxel_count_ += 1;
    }
    else
    {
      previous_voxel_unknown_ = false;
      current_ray_ig_ = 0;
      current_ray_voxel_count_ = 0;
    }
    p_vis_*=p_occ;
    
  }
  
}

}

}

#undef CSCOPE
#undef TEMPT