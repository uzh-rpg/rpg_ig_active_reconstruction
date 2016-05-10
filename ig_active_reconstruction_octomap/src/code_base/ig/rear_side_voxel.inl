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
#define CSCOPE RearSideVoxelIg<TREE_TYPE>


namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
namespace octomap
{  
  TEMPT
  CSCOPE::RearSideVoxelIg( Config utils )
  : utils_(utils)
  , rear_side_voxel_count_(0)
  , previous_voxel_unknown_(false)
  {
    
  }
  
  TEMPT
  std::string CSCOPE::type()
  {
    return "RearSideVoxelIg";
  }
  
  TEMPT
  typename CSCOPE::GainType CSCOPE::getInformation()
  {
    return rear_side_voxel_count_;
  }
  
  TEMPT
  void CSCOPE::makeReadyForNewRay()
  {
    previous_voxel_unknown_ = false;
  }
  
  TEMPT
  void CSCOPE::reset()
  {
    rear_side_voxel_count_ = 0;
    previous_voxel_unknown_ = false;
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
    // didn't hit anything -> no rear side voxel...
    previous_voxel_unknown_ = false;
  }
  
  TEMPT
  uint64_t CSCOPE::voxelCount()
  {
    return rear_side_voxel_count_;
  }
  
  TEMPT
  void CSCOPE::includeMeasurement( typename TREE_TYPE::NodeType* node )
  {
    if( node==NULL || !node->hasMeasurement() )
    {
      previous_voxel_unknown_=true;
      return;
    }
    
    double p_occ = utils_.pOccupancy(node);
    
    if( utils_.isUnknown(p_occ) )
    {
      previous_voxel_unknown_=true;
      return;
    }
    else
    {
      if( utils_.isOccupied(p_occ) && previous_voxel_unknown_ )
      {
	++rear_side_voxel_count_;
      }
      
      previous_voxel_unknown_=false;
    }
    
  }
  
}

}

}

#undef CSCOPE
#undef TEMPT