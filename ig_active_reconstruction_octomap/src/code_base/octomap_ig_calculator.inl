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
#define CSCOPE IgCalculator<TREE_TYPE>

#include <boost/function.hpp>

namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
namespace octomap
{ 
  
  /*TEMPT // postponed for compilation with cpp11
  template< template<typename> class IG_METRIC_TYPE, typename ... IG_CONSTRUCTOR_ARGS >
  unsigned int CSCOPE::registerInformationGain( IG_CONSTRUCTOR_ARGS ... args )
  {
    // gcc has a bug when capturing variadic arguments... need to use workaround...
    
    boost::shared_ptr< InformationGain<TREE_TYPE> > prototype = boost::make_shared< IG_METRIC_TYPE<TREE_TYPE> >(args...);
    std::string name = prototype->type();
    
    std::function< boost::shared_ptr< InformationGain<TREE_TYPE> >() > creator = std::bind(boost::make_shared< IG_METRIC_TYPE<TREE_TYPE> >,args...);
    
    return ig_factory_.add(name,creator);
  }*/
  
  TEMPT
  template<template<typename> class IG_METRIC_TYPE>
  unsigned int CSCOPE::registerInformationGain( typename IG_METRIC_TYPE<TREE_TYPE>::Utils::Config utils )
  {
    boost::shared_ptr< InformationGain<TREE_TYPE> > prototype = boost::make_shared< IG_METRIC_TYPE<TREE_TYPE> >(utils);
    std::string name = prototype->type();
    
    boost::function< boost::shared_ptr< InformationGain<TREE_TYPE> >() > creator;    
    creator = boost::bind(&IgCalculator<TREE_TYPE>::makeShared<IG_METRIC_TYPE>, this, utils);
    
    return ig_factory_.add(name,creator);
  }
  
  TEMPT
  template<template<typename> class IG_METRIC_TYPE>
  boost::shared_ptr< InformationGain<TREE_TYPE> > CSCOPE::makeShared(typename IG_METRIC_TYPE<TREE_TYPE>::Utils::Config utils)
  {
    return boost::shared_ptr< InformationGain<TREE_TYPE> >( new IG_METRIC_TYPE<TREE_TYPE>(utils) );
  }
  
}

}

}

#undef CSCOPE
#undef TEMPT