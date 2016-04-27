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

#define TEMPT template<class TREE_TYPE, class POINTCLOUD_TYPE>
#define CSCOPE WorldRepresentation<TREE_TYPE, POINTCLOUD_TYPE>

namespace ig_based_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  
  TEMPT
  CSCOPE::WorldRepresentation( typename TREE_TYPE::Config config )
  : octree_( std::make_shared<TREE_TYPE>(config) )
  , calculated_occlusions_(false)
  , occlusion_calculator_(false)
  {
    
  }
  
  TEMPT
  CSCOPE::~WorldRepresentation()
  {
    
  }
  
  TEMPT
  template< template<typename,typename> class OCCLUSION_CALC_TYPE, class ... Types >
  void CSCOPE::setOcclusionCalculator( Types ... args )
  {
    occlusion_calculator_ = std::make_shared< OCCLUSION_CALC_TYPE<TREE_TYPE,PclPointCloud> >( args... );
    occlusion_calculator_->setOctree(octree_);
  }
  
  TEMPT
  template< template<typename,typename> class INPUT_OBJ_TYPE, class ... Types >
  std::shared_ptr< INPUT_OBJ_TYPE<TREE_TYPE,POINTCLOUD_TYPE> > CSCOPE::getInputObj( Types ... args )
  {
    std::shared_ptr< INPUT_OBJ_TYPE<TREE_TYPE,PclPointCloud> > ptr = std::make_shared< INPUT_OBJ_TYPE<TREE_TYPE,PclPointCloud> >( args... );
    ptr->setOctree(octree_);
    
    return ptr;
  }
  
  TEMPT
  void CSCOPE::computeOcclusions( const Eigen::Vector3d& origin, const PclPointCloud& pcl )
  {
    if( occlusion_calculator_==nullptr )
      return;
    
    occlusion_calculator_->insert(origin,pcl);
    calculated_occlusions_ = true;
  }
  
  TEMPT
  bool CSCOPE::knowsOcclusions()
  {
    return calculated_occlusions_;
  }
  
}

}

}

#undef CSCOPE
#undef TEMPT