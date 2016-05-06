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

#define TEMPT template<class TREE_TYPE, class POINTCLOUD_TYPE>
#define CSCOPE PclInput<TREE_TYPE, POINTCLOUD_TYPE>

#include <limits>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

namespace ig_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  TEMPT
  void CSCOPE::setOctree( std::shared_ptr<TREE_TYPE> octree )
  {
    this->link_.octree = octree;
  }
  
  TEMPT
  template< template<typename,typename> class OCCLUSION_CALC_TYPE, class ... Types >
  void CSCOPE::setOcclusionCalculator( Types ... args )
  {
    occlusion_calculator_ = std::make_shared< OCCLUSION_CALC_TYPE<TREE_TYPE,POINTCLOUD_TYPE> >( args... );
    occlusion_calculator_->setLink(this->link_);
  }
}

}

}

#undef CSCOPE
#undef TEMPT