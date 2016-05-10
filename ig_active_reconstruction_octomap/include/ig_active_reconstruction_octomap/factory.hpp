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

#include <map>
#include <functional>
#include <memory>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

namespace multikit
{
  /*! Templated implementaiton of a factory providing name and id access.
   */
  template<class TYPE>
  class Factory
  {
  public:
    struct Entry
    {
      unsigned int id;
      std::string name;
      boost::function< boost::shared_ptr<TYPE>() > create;
    };
    
    typedef typename std::vector<Entry>::iterator Iterator;
    
    typedef TYPE Type;
    typedef boost::shared_ptr<TYPE> TypePtr;
    
  public:
    /*! Function to register object creation functions.
      * @param ig_name Name of the object type
      * @param ig_creator Function that returns a pointer to a new object of the corresponding type.
      * @return The unique id of the object type.
      */
    unsigned int add( std::string ig_name, boost::function< boost::shared_ptr<TYPE>() > ig_creator );
    
    /*! Function to create a new object of a specific type through its name.
      * If more than one object type registered themselves with the same name, the lastly registered one overwrites its predecessors.
      * @param name Name of the object type.
      * @return Pointer to a newly created object type instance 'name', nullptr if 'name' was not found.
      */
    boost::shared_ptr<TYPE> get(std::string name);
    
    /*! Function to create a new object of a specific type through its id.
      * Unlike its name, the id will be unique (unless an overflow occurs for the id type... ;) )
      * @param id Id of the object type.
      * @return Poitner to a newly created object type instance with the given id.
      */
    boost::shared_ptr<TYPE> get(unsigned int id);
    
    /*! Returns the name corresponding to an id.
     * @throws std::invalid_argument if the id is unknown
      */
    std::string nameOf(unsigned int id);
    
    /*! Returns the id corresponding to a name.
     * @throws std::invalid_argument if the name is unknown
      */
    unsigned int idOf(std::string name);
    
    /*! Iterators to iterate through entries...
      */
    Iterator begin();
    
    /*! Iterators to iterator through entries...
      */
    Iterator end();
  private:
    std::vector<Entry> entries_; //! All entries... The id corresponds directly to the position in the vector.
  };
}

#include "../src/code_base/factory.inl"