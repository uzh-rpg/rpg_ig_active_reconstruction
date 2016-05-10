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

#include <octomap/OccupancyOcTreeBase.h>

#include "ig_active_reconstruction_octomap/octomap_ig_tree_node.hpp"

namespace ig_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  
  /*! Occupancy OcTree class that uses our IgTreeNode class
   * as node type
   */
  class IgTree: public ::octomap::OccupancyOcTreeBase<IgTreeNode>
  {
  public:
    typedef IgTreeNode NodeType;
    
    /*! Configuration for the IgTree
     */
    struct Config
    {
    public:
      Config();
      
    public:
      double resolution_m; //! OcTree leaf node size, default: 0.1 [m].
      double occupancy_threshold; //! Occupancy probability over which nodes are considered occupied, default: 0.5 [range 0-1].
      double hit_probability; //! Probability update value for hits, default 0.7 [range 0-1].
      double miss_probability; //! Probability update value for misses, default 0.4 [range 0-1].
      double clamping_threshold_min; //! Min probability threshold over which the probability is clamped, default: 0.12, range [0-1].
      double clamping_threshold_max; //! Max probability threshold over which the probability is clamped, default: 0.97, range [0-1].
    };
    
  public:
    //! Default constructor, sets resolution of leafs
    IgTree(double resolution_m);
    
    /*! Constructor with complete configuration
     */
    IgTree(Config config);

    /*! virtual constructor: creates a new object of same type
     * (Covariant return type requires an up-to-date compiler)
     */
    IgTree* create() const;

    std::string getTreeType() const;
    
    /*! Returns the current configuration.
     */
    const Config& config() const;
    
  protected:
    /*! Sets octree options based on current configuration
     */
    void updateOctreeConfig();
    
  protected:
    Config config_;


  protected:
    /*!
    * Static member object which ensures that this OcTree's prototype
    * ends up in the classIDMapping only once
    */
    class StaticMemberInitializer{
    public:
    StaticMemberInitializer() {
	IgTree* tree = new IgTree(0.1);
	AbstractOcTree::registerTreeType(tree);
    }
    };
    //! to ensure static initialization (only once)
    static StaticMemberInitializer igTreeMemberInit;

  };
  
  
}

}

}