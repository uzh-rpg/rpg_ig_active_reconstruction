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

#define TEMPT template<class TREE_TYPE>
#define CSCOPE WorldRepresentation<TREE_TYPE>

namespace ig_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  
  TEMPT
  CSCOPE::WorldRepresentation( typename TREE_TYPE::Config config )
  : octree_( std::make_shared<TREE_TYPE>(config) )
  {
    
  }
  
  TEMPT
  CSCOPE::~WorldRepresentation()
  {
    
  }
  
  TEMPT
  template< template<typename,typename ...> class INPUT_OBJ_TYPE, class ... TEMPLATE_ARGS, class ... CONSTRUCTOR_ARGS >
  std::shared_ptr< INPUT_OBJ_TYPE<TREE_TYPE,TEMPLATE_ARGS ...> > CSCOPE::getLinkedObj( CONSTRUCTOR_ARGS ... args )
  {
    std::shared_ptr< INPUT_OBJ_TYPE<TREE_TYPE,TEMPLATE_ARGS ...> > ptr = std::make_shared< INPUT_OBJ_TYPE<TREE_TYPE,TEMPLATE_ARGS ...> >( args... );
    
    Link new_link;
    new_link.octree = octree_;
    ptr->setLink(new_link);
    
    return ptr;
  }
}

}

}

#undef CSCOPE
#undef TEMPT