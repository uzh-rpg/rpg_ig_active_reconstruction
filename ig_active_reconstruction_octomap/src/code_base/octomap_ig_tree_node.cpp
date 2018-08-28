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

  bool IgTreeNode::operator==(const IgTreeNode& rhs) const
  {
    return rhs.value == value && rhs.occ_dist_ == occ_dist_ && rhs.has_no_measurement_==has_no_measurement_;
  }

  void IgTreeNode::copyData(const IgTreeNode& from){
    value = from.value;
    occ_dist_ = from.occ_dist_;
    max_dist_ = from.max_dist_;
    has_no_measurement_ = from.has_no_measurement_;
  }

  // ============================================================
  // =  occupancy probability  ==================================
  // ============================================================

  double IgTreeNode::getMeanChildLogOdds() const
  {
    double mean = 0;
    char c = 0;
    if(children != NULL)
    {
      for (unsigned int i=0; i<8; i++)
      {
        IgTreeNode* child = static_cast<IgTreeNode*>(children[i]);
        if (child != NULL)
        {
          mean += child->getOccupancy();
          c++;
        }
      }
    }
    if (c)
      mean /= (double) c;

    return log(mean/(1-mean));
  }

  float IgTreeNode::getMaxChildLogOdds() const
  {
    float max = -std::numeric_limits<float>::max();
    if(children != NULL)
    {
      for (unsigned int i=0; i<8; i++)
      {
        IgTreeNode* child = static_cast<IgTreeNode*>(children[i]);
        if (child != NULL)
        {
          float l = child->getLogOdds();
          if (l > max)
            max = l;
        }
      }
    }
    return max;
  }

  double IgTreeNode::getMinChildOccDist() const
  {
    double min = std::numeric_limits<double>::max();
    if(children != NULL)
    {
      for(unsigned int i=0; i<8; ++i)
      {
        IgTreeNode* child = static_cast<IgTreeNode*>(children[i]);
        if(child != NULL)
        {
          double d = child->occDist();
          if(d < min)
            min = d;
        }
      }
    }
    return min;
  }

  double IgTreeNode::getMaxChildDist() const
  {
    double minmax = std::numeric_limits<double>::max();
    if(children != NULL)
    {
      for (unsigned int i=0; i<8; i++)
      {
        IgTreeNode* child = static_cast<IgTreeNode*>(children[i]);
        if (child != NULL)
        {
          double d = child->maxDist();
          if (d < minmax)
            minmax = d;
        }
      }
    }
    return minmax;
  }

  void IgTreeNode::addValue(const float& logOdds)
  {
    value += logOdds;
  }

}

}

}
