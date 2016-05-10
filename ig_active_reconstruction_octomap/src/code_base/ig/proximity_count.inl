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
#define CSCOPE ProximityCountIg<TREE_TYPE>


namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
namespace octomap
{  
  TEMPT
  CSCOPE::ProximityCountIg( Config utils )
  : utils_(utils)
  , ig_(0)
  , voxel_count_(0)
  {
    
  }
  
  TEMPT
  std::string CSCOPE::type()
  {
    return "ProximityCountIg";
  }
  
  TEMPT
  typename CSCOPE::GainType CSCOPE::getInformation()
  {
    return ig_;
  }
  
  TEMPT
  void CSCOPE::makeReadyForNewRay()
  {
  }
  
  TEMPT
  void CSCOPE::reset()
  {
    ig_ = 0;
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
    // can't tell what occlusions might be in there...
  }
  
  TEMPT
  uint64_t CSCOPE::voxelCount()
  {
    return voxel_count_;
  }
  
  TEMPT
  void CSCOPE::includeMeasurement( typename TREE_TYPE::NodeType* node )
  {
    if( node!=NULL )
    {
      double dist = node->occDist();
      if( !node->hasMeasurement() && dist>0 )
      {
	ig_ += node->maxDist()-dist;
	++voxel_count_;
      }	
    }
    
    return;
  }
  
}

}

}

#undef CSCOPE
#undef TEMPT