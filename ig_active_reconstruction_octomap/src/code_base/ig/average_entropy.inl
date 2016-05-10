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
#define CSCOPE AverageEntropyIg<TREE_TYPE>


namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
namespace octomap
{  
  TEMPT
  CSCOPE::AverageEntropyIg( Config utils )
  : utils_(utils)
  , voxel_count_(0)
  , total_ig_(0)
  , current_ray_voxels_(0)
  , current_ray_entropy_(0)
  {
    
  }
  
  TEMPT
  std::string CSCOPE::type()
  {
    return "AverageEntropyIg";
  }
  
  TEMPT
  typename CSCOPE::GainType CSCOPE::getInformation()
  {
    if( voxel_count_==0 )
    {
      return 0;
    }
    return total_ig_/voxel_count_;
  }
  
  TEMPT
  void CSCOPE::makeReadyForNewRay()
  {
    current_ray_voxels_ = 0;
    current_ray_entropy_ = 0;
  }
  
  TEMPT
  void CSCOPE::reset()
  {
    voxel_count_ = 0;
    total_ig_ = 0;
    current_ray_voxels_ = 0;
    current_ray_entropy_ = 0;
  }
  
  TEMPT
  void CSCOPE::includeRayMeasurement( typename TREE_TYPE::NodeType* node )
  {
    includeMeasurement(node);
  }
  
  TEMPT
  void CSCOPE::includeEndPointMeasurement( typename TREE_TYPE::NodeType* node )
  {
    double occ = utils_.pOccupancy(node);
    double ent = utils_.entropy(occ);
    
    ++current_ray_voxels_;
    current_ray_entropy_ += ent;
    
    // only include rays that hit an occupied!
    if( utils_.isOccupied(occ) )
    {
      total_ig_ += current_ray_entropy_;
      voxel_count_ += current_ray_voxels_;
    }
  }
  
  TEMPT
  void CSCOPE::informAboutVoidRay()
  {
    // didn't hit anything -> no rear side voxel...
  }
  
  TEMPT
  uint64_t CSCOPE::voxelCount()
  {
    return voxel_count_;
  }
  
  TEMPT
  void CSCOPE::includeMeasurement( typename TREE_TYPE::NodeType* node )
  {
    double occ = utils_.pOccupancy(node);
    double ent = utils_.entropy(occ);
    
    ++current_ray_voxels_;
    current_ray_entropy_ += ent;
  }
  
}

}

}

#undef CSCOPE
#undef TEMPT