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

#define TEMPT template<class TYPE>
#define CSCOPE Factory<TYPE>

#include <stdexcept>
#include <sstream>
#include <boost/foreach.hpp>

namespace multikit
{
  
  TEMPT
  unsigned int CSCOPE::add( std::string ig_name, boost::function< boost::shared_ptr<TYPE>()> ig_creator )
  {
    Entry new_entry;
    new_entry.id = entries_.size();
    new_entry.name = ig_name;
    new_entry.create = ig_creator;
    
    entries_.push_back(new_entry);
    
    return new_entry.id;
  }
  
  TEMPT
  boost::shared_ptr<TYPE> CSCOPE::get(std::string name)
  {
    boost::shared_ptr<TYPE> new_object;
    BOOST_FOREACH( Entry& entry, entries_ )
    {
      if( entry.name==name )
	new_object = entry.create();
    }
    
    return new_object;
  }
  
  TEMPT
  boost::shared_ptr<TYPE> CSCOPE::get(unsigned int id)
  {
    if( id>=entries_.size() )
    {
      return boost::shared_ptr<TYPE>();
    }
    return entries_[id].create();
  }
  
  TEMPT
  std::string CSCOPE::nameOf(unsigned int id)
  {
    if( id>=entries_.size() )
    {
      std::stringstream error_desc;
      error_desc<<"Factory<TYPE>::nameOf:: Passed argument id = "<<id<<" is invalid.";
      throw std::invalid_argument( error_desc.str() );
    }
    else
    {
      return entries_[id].name;
    }
  }
  
  TEMPT
  unsigned int CSCOPE::idOf(std::string name)
  {
    BOOST_FOREACH( Entry& entry, entries_ )
    {
      if( entry.name==name )
      {
	return entry.id;
      }
    }
    // not found
    std::stringstream error_desc;
    error_desc<<"Factory<TYPE>::idOf:: Passed argument name = "<<name<<" is invalid.";
    throw std::invalid_argument( error_desc.str() );
  }
  
  TEMPT
  typename CSCOPE::Iterator CSCOPE::begin()
  {
    return entries_.begin();
  }
  
  TEMPT
  typename CSCOPE::Iterator CSCOPE::end()
  {
    return entries_.end();
  }
  
}

#undef CSCOPE
#undef TEMPT