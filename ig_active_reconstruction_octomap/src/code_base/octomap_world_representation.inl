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
#define CSCOPE WorldRepresentation<TREE_TYPE>

namespace ig_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  
  TEMPT
  CSCOPE::WorldRepresentation( typename TREE_TYPE::Config config )
  : octree_( boost::make_shared<TREE_TYPE>(config) )
  {
    
  }
  
  TEMPT
  CSCOPE::~WorldRepresentation()
  {
    
  }
  
  /*TEMPT // cpp11 version
  template< template<typename, typename ...> class INPUT_OBJ_TYPE, class ... TEMPLATE_ARGS, class ... CONSTRUCTOR_ARGS >
  boost::shared_ptr< INPUT_OBJ_TYPE<TREE_TYPE,TEMPLATE_ARGS ...> > CSCOPE::getLinkedObj( CONSTRUCTOR_ARGS ... args )
  {
    boost::shared_ptr< INPUT_OBJ_TYPE<TREE_TYPE,TEMPLATE_ARGS ...> > ptr = boost::make_shared< INPUT_OBJ_TYPE<TREE_TYPE,TEMPLATE_ARGS ...> >( args... );
    
    Link new_link;
    new_link.octree = octree_;
    ptr->setLink(new_link);
    
    return ptr;
  }
  
  TEMPT // cpp11 version
  template< template<typename> class INPUT_OBJ_TYPE, class ... CONSTRUCTOR_ARGS >
  boost::shared_ptr< INPUT_OBJ_TYPE<TREE_TYPE> > CSCOPE::getLinkedObj( CONSTRUCTOR_ARGS ... args )
  {
    boost::shared_ptr< INPUT_OBJ_TYPE<TREE_TYPE> > ptr = boost::make_shared< INPUT_OBJ_TYPE<TREE_TYPE> >( args... );
    
    Link new_link;
    new_link.octree = octree_;
    ptr->setLink(new_link);
    
    return ptr;
  }*/
  
  TEMPT
  template< template<typename> class INPUT_OBJ_TYPE>
  boost::shared_ptr< INPUT_OBJ_TYPE<TREE_TYPE> > CSCOPE::getLinkedObj( typename INPUT_OBJ_TYPE<TREE_TYPE>::Config config )
  {
    boost::shared_ptr< INPUT_OBJ_TYPE<TREE_TYPE> > ptr = boost::make_shared< INPUT_OBJ_TYPE<TREE_TYPE> >( config );
    
    Link new_link;
    new_link.octree = octree_;
    ptr->setLink(new_link);
    
    return ptr;
  }
  
  TEMPT
  template< template<typename> class INPUT_OBJ_TYPE>
  boost::shared_ptr< typename INPUT_OBJ_TYPE<TREE_TYPE>::Type > CSCOPE::getLinkedObj( typename INPUT_OBJ_TYPE<TREE_TYPE>::Type::Config config )
  {
    boost::shared_ptr< typename INPUT_OBJ_TYPE<TREE_TYPE>::Type > ptr = boost::make_shared< typename INPUT_OBJ_TYPE<TREE_TYPE>::Type >( config );
    
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