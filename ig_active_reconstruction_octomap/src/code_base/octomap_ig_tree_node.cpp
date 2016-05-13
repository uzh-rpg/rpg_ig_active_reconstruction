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


#include "ig_active_reconstruction_octomap/octomap_ig_tree_node.hpp"
#include <limits>

namespace ig_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  
  IgTreeNode::IgTreeNode()
  : ::octomap::OcTreeNode()
  , occ_dist_(-1)
  , max_dist_(std::numeric_limits<double>::max())
  , has_no_measurement_(false)
  {
  }

  IgTreeNode::~IgTreeNode()
  {
  }
  
  void IgTreeNode::expandNode()
  {
    assert(!hasChildren());

    for (unsigned int k=0; k<8; k++) {
      createChild(k);
      children[k]->setValue(value);
    }
  }

  // TODO: Use Curiously Recurring Template Pattern instead of copying full function
  // (same for getChild)
  bool IgTreeNode::createChild(unsigned int i)
  {
    if (children == NULL) {
      allocChildren();
    }
    assert (children[i] == NULL);
    children[i] = new IgTreeNode();
    return true;
  }

  // ============================================================
  // =  occupancy probability  ==================================
  // ============================================================

  double IgTreeNode::getMeanChildLogOdds() const
  {
    double mean = 0;
    char c = 0;
    for (unsigned int i=0; i<8; i++)
    {
      if (childExists(i))
      {
        mean += getChild(i)->getOccupancy();
        c++;
      }
    }
    if (c)
      mean /= (double) c;

    return log(mean/(1-mean));
  }

  float IgTreeNode::getMaxChildLogOdds() const
  {
    float max = -std::numeric_limits<float>::max();
    for (unsigned int i=0; i<8; i++)
    {
      if (childExists(i))
      {
        float l = getChild(i)->getLogOdds();
        if (l > max)
          max = l;
      }
    }
    return max;
  }

  void IgTreeNode::addValue(const float& logOdds)
  {
    value += logOdds;
  }
  
}

}

}