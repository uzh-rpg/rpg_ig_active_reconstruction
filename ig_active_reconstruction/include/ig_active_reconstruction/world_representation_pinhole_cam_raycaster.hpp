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

#include "ig_active_reconstruction/world_representation_raycaster.hpp"

namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
  class PinholeCamRayCaster: public RayCaster
  {
  public:
    
    struct ResolutionSettings
    {
    public:
      /*! Constructor sets default settings. */
      ResolutionSettings();
      
      /*! Comparison operator, only one needed atm.
       */
      bool operator!=( ResolutionSettings& comp ) const;
      
    public:
      double ray_resolution_x; //! How many rays are cast per pixel on the image's x-axis to obtain the information. [rays/px] Default: 1.0
      double ray_resolution_y; //! How many rays are cast per pixel on the image's y-axis to obtain the information. [rays/px] Default: 1.0
      double min_x_perc; //! Minimum x to be cast [%of image width]. Default:0.
      double min_y_perc; //! Minimum y to be cast [%of image height]. Default:0.
      double max_x_perc; //! Maximum x to be cast [%of image width]. Default:1.
      double max_y_perc; //! Maximum y to be cast [%of image height]. Default:1.
    };
    
    /*! Configuration structure.
     */
    struct Config
    {
    public:
      /*! Constructor sets default values.
       */
      Config();
      
    public:
      ResolutionSettings resolution; //! Settings of the resolution.
      double max_ray_depth_m; //! Default: 1m
      unsigned int img_width_px; //! Image width [px]. Default: 0.
      unsigned int img_height_px; //! Image height [px]. Default: 0.
      Eigen::Matrix3d camera_matrix; //! (Intrinsic) camera matrix [px]. Default: Identity.
    };
    
  public:
    /*! Constructor.
     */
    PinholeCamRayCaster( Config config = Config() );
    
    /*! Sets new resolution settings.
     */
    void setResolution( ResolutionSettings& res_settings );
    
    /*! Sets new configuration.
     */
    void setConfig( Config config );
    
    /*! Projects 2d-image coordinates to 3d-ray direction using the camera matrix. 
     * The direction is normalized.
     * 
     * @param x_px x-coordinate [px].
     * @param y_px y-coordiante [px].
     */
    RayDirection projectPixelTo3dRay( unsigned int x_px, unsigned int y_px );
    
    /*! Returns a set of rays cast from sensor_pose with the current configuration
     * @param sensor_pose Position from which rays are cast.
     * @return Pointer to a set of rays.
     */
    virtual boost::shared_ptr<RaySet> getRaySet( movements::Pose& sensor_pose );
    
    /*! Returns the set of ray directions as the would be cast from the given sensor_pose with the current configuration.
     * @param sensor_pose Position from which rays are cast.
     * @return Pointer to a set of ray directions.
     */
    virtual boost::shared_ptr<RayDirectionSet> getRayDirectionSet( movements::Pose& sensor_pose );
    
    /*! Returns the set of ray directions as cast for the current configuration, relative to the sensor.
     */
    virtual boost::shared_ptr<const RayDirectionSet> getRelRayDirectionSet() const;
    
  protected:
    /*! (Re-)computes the internal camera coordinate relative ray direction set, given
     * the current configuration.
     */
    void computeRelRayDirections();
    
  protected:
    Config config_; //! Configuration.
    
    boost::shared_ptr<RayDirectionSet> ray_directions_; //! Precomputed set of ray directions relative to the camera (sensor) coordinate frame.
  };
  
}

}