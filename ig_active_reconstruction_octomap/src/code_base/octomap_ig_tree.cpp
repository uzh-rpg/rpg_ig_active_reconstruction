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


#include "ig_active_reconstruction_octomap/octomap_ig_tree.hpp"


namespace ig_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  IgTree::Config::Config()
  : resolution_m(0.1)
  , occupancy_threshold(0.5)
  , hit_probability(0.7)
  , miss_probability(0.4)
  , clamping_threshold_min(0.12)
  , clamping_threshold_max(0.97)
  {
    
  }
  
  IgTree::IgTree(double resolution_m)
  : ::octomap::OccupancyOcTreeBase<IgTreeNode>(resolution_m)
  {
    config_.resolution_m = resolution_m;
    updateOctreeConfig();
  }
  
  IgTree::IgTree(Config config)
  : ::octomap::OccupancyOcTreeBase<IgTreeNode>(config.resolution_m)
  , config_(config)
  {
    updateOctreeConfig();
  }
  
  IgTree* IgTree::create() const
  {
    return new IgTree(config_);
  }
  
  const IgTree::Config& IgTree::config() const
  {
    return config_;
  }
  
  void IgTree::updateOctreeConfig()
  {
    setOccupancyThres(config_.occupancy_threshold);
    setProbHit(config_.hit_probability);
    setProbMiss(config_.miss_probability);
    setClampingThresMin(config_.clamping_threshold_min);
    setClampingThresMax(config_.clamping_threshold_max);
  }
  
  std::string IgTree::getTreeType() const
  {
    return "IgTree";
  }
  
}

}

}