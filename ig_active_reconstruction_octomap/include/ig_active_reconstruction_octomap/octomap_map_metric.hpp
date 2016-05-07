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

#pragma once

namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
namespace octomap
{  
  
  /*! Defines the interface for and a factory to register map metrics. A map metric is a metric that
   * is calculated on the complete map (octree), not only on rays cast through the space. One example
   * is the total entropy within the map.
   */
  template<class TREE_TYPE>
  class MapMetric
  {
  public:
    typedef double Result;
    
    
  public:
    /*! Returns the name of the method.
     */
    virtual std::string type()=0;
    
    /*! Calculates the metric on the given octree.
     * @param octree Pointer to the octree on which the metric will be calculated.
     */
    Result calculateOn( boost::shared_ptr<TREE_TYPE> octree )=0;
  };
  
}

}

}

#include "../src/code_base/octomap_map_metric.inl"