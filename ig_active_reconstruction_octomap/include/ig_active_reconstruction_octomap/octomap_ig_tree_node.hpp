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


#include <octomap/octomap_types.h>
#include <octomap/octomap_utils.h>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>


#include <limits>

namespace ig_active_reconstruction
{

namespace world_representation
{

namespace octomap
{

  // forward declaraton for "friend"
  class IgTree;

  /*!
   * The IgTreeNode is based on octomaps OcTreeNode class, adding
   * some functionality needed for information gain calculations.
   *
   */
  //class IgTreeNode : public ::octomap::OcTreeNode
  class IgTreeNode : public ::octomap::OcTreeNode
  {

  public:
    friend class IgTree; // needs access to node children (inherited)

    IgTreeNode();
    ~IgTreeNode();

    bool operator==(const IgTreeNode& rhs) const;

    void copyData(const IgTreeNode& from);

    // -- node occupancy  ----------------------------

    /// \return occupancy probability of node
    inline double getOccupancy() const { return ::octomap::probability(value); }

    /// \return log odds representation of occupancy probability of node
    inline float getLogOdds() const{ return value; }
    /// sets log odds occupancy of node
    inline void setLogOdds(float l) { value = l; }

    /**
     * @return mean of all children's occupancy probabilities, in log odds
     */
    double getMeanChildLogOdds() const;

    /**
     * @return maximum of children's occupancy probabilities, in log odds
     */
    float getMaxChildLogOdds() const;

    double getMinChildOccDist() const;
    double getMaxChildDist() const;

    /// adds p to the node's logOdds value (with no boundary / threshold checking!)
    void addValue(const float& p);


    double occDist(){return occ_dist_;};
    // sets occDist if it's smaller than the previous value
    void updateOccDist( double occDist )
    {
	    if(occ_dist_==-1)
	      occ_dist_=occDist;
	    else
	      occ_dist_=std::min(occ_dist_,occDist);

    };

    double maxDist(){return max_dist_;};
    void setMaxDist(double max_dist){max_dist_=max_dist;};

    // whether this node has been measured or not
    bool hasMeasurement(){return !has_no_measurement_;};
    void updateHasMeasurement( bool hasMeasurement ){has_no_measurement_=!hasMeasurement;};

  protected:
    double occ_dist_; //! if node is occluded this sets the shortest distance from an occupied node for which the occlusion was registered, -1 if not registered so far
    double max_dist_; //! Maximal occlusion update distance used when calculating occlusions.
    bool has_no_measurement_; //! True if this node was setup for additional data but was not actually part of a measurement (not free and not occupied)
  };

} // end namespace

}

}
