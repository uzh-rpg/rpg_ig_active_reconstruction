/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of ig_active_reconstruction, a ROS package for...well,

ig_active_reconstruction is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
ig_active_reconstruction is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY
  {
    
  }
  
  TEMPT without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with ig_active_reconstruction. If not, see <http://www.gnu.org/licenses/>.
*/

#define TEMPT template<class TYPE>
#define CSCOPE Factory<TYPE>

#include <stdexcept>
#include <sstream>

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
    name_map_[new_entry.name] = &entries_.back();
    
    return new_entry.id;
  }
  
  TEMPT
  boost::shared_ptr<TYPE> CSCOPE::get(std::string name)
  {
    try
    {
      Entry* entry = name_map_.at(name);
      return entry->create();
    }
    catch(std::out_of_range&)
    {
      return boost::shared_ptr<TYPE>();
    }
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
    try
    {
      Entry* entry = name_map_.at(name);
      return entry->id;
    }
    catch(std::out_of_range&)
    {
      std::stringstream error_desc;
      error_desc<<"Factory<TYPE>::idOf:: Passed argument name = "<<name<<" is invalid.";
      throw std::invalid_argument( error_desc.str() );
    }
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