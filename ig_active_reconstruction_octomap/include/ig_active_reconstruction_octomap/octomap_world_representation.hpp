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

#pragma once

namespace ig_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  /*! Templated wrapper class around the octree object. Currently it mainly provides some convenience functionality
   * to get different interface classes to work with the octree without having to respecify the correct
   * template parameters each time. Also give the possibility to possibly add more functionality in the future.
   */
  template<class TREE_TYPE>
  class WorldRepresentation
  {
  public:
    //! Make tree type available
    typedef TREE_TYPE TreeType;
    
    /*! The link structure is used to link objects with the octomap world representation.
     */
    struct Link
    {
      boost::shared_ptr<TREE_TYPE> octree;
    };
    
    /*! Base class providing "link-functionality"
     */
    class LinkedObject
    {
    public:
      /*! Links to a WorldRepresentation object, among others sets the octree to the one encapsulated in the world representation.
      */
      virtual void setLink( Link& link ){ link_=link; };
      
    protected:
      Link link_; //! WorldRepresentation link
    };
    
  public:
    WorldRepresentation( typename TREE_TYPE::Config config = typename TREE_TYPE::Config() );
    
    virtual ~WorldRepresentation();
    
    /*! (cpp11 version)Returns a shared pointer to an object on which a setLink() was called, with a link object linking to the world representation. 
     * The type of the object is the first template parameter of the function. It must be a templated type where the first template argument is
     * the TREE_TYPE. It is automatically templated on the TREE_TYPE used within the world representation. If the linked object expects
     * other template arguments apart from the tree type, those can be appended as further function template parameters. Whatever inputs necessary
     * as constructor argument for the object type can be passed as arguments to the function.
     * 
     * Example:
     * ******************************************
     * StdPclInput is a class to feed pcl pointclouds into the octomap octree to which it is linked. It is templated on octree and pointcloud types and takes a 
     * StdPclInput::Config object as optional constructor argument.
     * 
     * Given an already existing "config" object it could be created through:
     * auto std_input = std::make_shared< StdPclInput<IgTree,pcl::PointCloud<pcl::PointXYZ>> >(config);
     * 
     * and then linked to a WorldRepresentation<IgTree> "world" through:
     * WorldRepresentation<IgTree>::Link link = world.getLink();
     * std_input.setLink(link);
     * 
     * This can be shortcut with this function, while at the same time automatically deducing the TREE_TYPE (IgTree in the example) of the world representation
     * to which the StdPclInput is to be linked. The following call is equivalent to the above three commands:
     * auto std_input = world.getLinkedObj<StdPclInput, pcl::PointCloud<pcl::PointXYZ> >(config);
     *
     * @tparam INPUT_OBJ_TYPE Class type of the object for which a shared_ptr shall be created. Must be templated on at least one argument, where the first one must be the tree type.
     * @tparam TEMPLATE_ARGS (variadic) Further optional template arguments of the INPUT_OBJ_TYPE class succeeding the tree type
     * @tparam CONSTRUCTOR_ARGS (variadic) These do not have to be specified but will be deduced by optionally provided function arguments args
     * @param args Whichever arguments the input object type expects. (variadic template)
     * @return Shared pointer to a newly instantiated object of type INPUT_OBJ_TYPE<TREE_TYPE,TEMPLATE_ARGS...>, instantiated with args passed to the constructor. Also its setLink()-function is called.
     */
    /*template< template<typename, typename ...> class INPUT_OBJ_TYPE, class ... TEMPLATE_ARGS, class ... CONSTRUCTOR_ARGS >
    boost::shared_ptr< INPUT_OBJ_TYPE<TREE_TYPE,TEMPLATE_ARGS ...> > getLinkedObj( CONSTRUCTOR_ARGS ... args );*/
    
    /*! (cpp11 version)Function overload TODO: Can't the above template function be adapted for class that only takes one template argument?! -> apparently might be a compiler bug
     */
    /*template< template<typename> class INPUT_OBJ_TYPE, class ... CONSTRUCTOR_ARGS >
    boost::shared_ptr< INPUT_OBJ_TYPE<TREE_TYPE> > getLinkedObj( CONSTRUCTOR_ARGS ... args );*/
    
    /*! Returns a shared pointer to an object on which a setLink() was called, with a link object linking to the world representation. 
     * The type of the object is the first template parameter of the function. It must be a templated type with exactly one template argument: the tree type, which is set automatically, derived from the object.
     * It must have a member type config which it takes as a constructor argument.
     */
    template< template<typename> class INPUT_OBJ_TYPE>
    boost::shared_ptr< INPUT_OBJ_TYPE<TREE_TYPE> > getLinkedObj( typename INPUT_OBJ_TYPE<TREE_TYPE>::Config config = typename INPUT_OBJ_TYPE<TREE_TYPE>::Config() );
    
    /*!  (cpp03) Returns a shared pointer to an object on which a setLink() was called, with a link object linking to the world representation. For templated objects with a partial specialization, e.g. StdPclInputPointXYZ<TREE_TYPE>::Type.
     */
    template< template<typename> class INPUT_OBJ_TYPE>
    boost::shared_ptr< typename INPUT_OBJ_TYPE<TREE_TYPE>::Type > getLinkedObj( typename INPUT_OBJ_TYPE<TREE_TYPE>::Type::Config config = typename INPUT_OBJ_TYPE<TREE_TYPE>::Type::Config() );
    
  protected:
    boost::shared_ptr<TREE_TYPE> octree_; //! Octomap tree instance.
  };
  
}

}

}

#include "../src/code_base/octomap_world_representation.inl"