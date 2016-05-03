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
#define CSCOPE IgCalculator<TREE_TYPE>

namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
namespace octomap
{ 
  
  TEMPT
  template< template<typename> class IG_METRIC_TYPE, typename ... IG_CONSTRUCTOR_ARGS >
  unsigned int CSCOPE::registerInformationGain( IG_CONSTRUCTOR_ARGS ... args )
  {
    // gcc has a bug when capturing variadic arguments... need to use workaround...
    
    std::shared_ptr< InformationGain<TREE_TYPE> > prototype = std::make_shared< IG_METRIC_TYPE<TREE_TYPE> >(args...);
    std::string name = prototype->type();
    
    std::function< std::shared_ptr< InformationGain<TREE_TYPE> >() > creator = std::bind(std::make_shared< IG_METRIC_TYPE<TREE_TYPE> >,args...);
    
    return ig_factory_.add(name,creator);
  }
  
}

}

}

#undef CSCOPE
#undef TEMPT