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

#include "ig_active_reconstruction_octomap/octomap_std_pcl_input.hpp"

namespace ig_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  // (cpp03 version...)
  template<class TREE_TYPE>
  struct StdPclInputPointXYZ
  {
    typedef pcl::PointCloud<pcl::PointXYZ> PclType;
    typedef TREE_TYPE TreeType;
    typedef StdPclInput< TreeType, PclType > Type;
    typedef boost::shared_ptr< StdPclInput< TreeType, PclType > > Ptr;
  };
  
}

}

}