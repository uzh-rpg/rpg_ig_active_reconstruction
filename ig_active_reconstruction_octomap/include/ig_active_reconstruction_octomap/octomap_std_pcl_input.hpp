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

#include <pcl/common/projection_matrix.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ig_active_reconstruction_octomap/octomap_pcl_input.hpp"

namespace ig_active_reconstruction
{
  
namespace world_representation
{

namespace octomap
{
  /*! Input object to feed pointclouds into an octomap::WorldRepresentation. It follows the standard octomap
   * way of doing so, refer to the dedicated paper. Easiest way to retrieve the object is to call getInputObj<StdPclInput>(config) on the
   * WorldRepresentation object. This will directly set the correct template arguments to interact with it.
   */
  template<class TREE_TYPE, class POINTCLOUD_TYPE>
  class StdPclInput: public PclInput<TREE_TYPE,POINTCLOUD_TYPE>
  {
  public:
    typedef boost::shared_ptr< StdPclInput<TREE_TYPE,POINTCLOUD_TYPE> > Ptr;
    typedef TREE_TYPE TreeType;
    typedef POINTCLOUD_TYPE PclType;
    
    struct Config
    {
    public:
      /*! Constructor sets default parameter values. */
      Config();
      
    public:
      bool use_bounding_box; //! Whether incoming pointclouds should be filtered or not using the configured bounding box. Default:false.
      ::octomap::point3d bounding_box_min_point_m; //! Defines bounding box minimum. Points with smaller coordinates are discarded, default: lowest double possible [m].
      ::octomap::point3d bounding_box_max_point_m; //! Defines bounding box maximum. Points with larger coordinates are discarded, default: largest double possible [m].
      double max_sensor_range_m; //! Maximal range for integrating sensor data [m]. Anything exceeding this distance will be dropped. For negative values, it is ignored. Default: -1.
    };
    
  public:
    /*! Constructor
     * @param config Configuration.
     */
    StdPclInput( Config config = Config() );
    
    /*! Virtual destructor.
     */
    ~StdPclInput(){};
    
    /*! Inserts a new pointcloud. If an occlusion calculator was set, it is called at the end.
     * 
     * @param sensor_to_world Transform from sensor to world coordinates.
     * @param pcl The pointcloud that is to be inserted, in sensor coordinates. Note that the function will operate directly on it
     */
    virtual void push( const Eigen::Transform<double,3,Eigen::Affine>& sensor_to_world, POINTCLOUD_TYPE& pcl );
    
  protected:
    Config config_;
  };
  
}

}

}

#include "../src/code_base/octomap_std_pcl_input.inl"