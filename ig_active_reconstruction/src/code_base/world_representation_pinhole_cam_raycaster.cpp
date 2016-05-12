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

#include "ig_active_reconstruction/world_representation_pinhole_cam_raycaster.hpp"

#include <boost/smart_ptr.hpp>

namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
  PinholeCamRayCaster::ResolutionSettings::ResolutionSettings()
  : ray_resolution_x(1.0)
  , ray_resolution_y(1.0)
  , min_x_perc(0)
  , min_y_perc(0)
  , max_x_perc(1)
  , max_y_perc(1)
  {
    
  }
  
  bool PinholeCamRayCaster::ResolutionSettings::operator!=( ResolutionSettings& comp ) const
  {
    return ray_resolution_x!=comp.ray_resolution_x
	|| ray_resolution_y!=comp.ray_resolution_y
	|| min_x_perc!=comp.min_x_perc
	|| min_y_perc!=comp.min_y_perc
	|| max_x_perc!=comp.max_x_perc
	|| max_y_perc!=comp.max_y_perc;
  }
  
  
  PinholeCamRayCaster::Config::Config()
  : resolution()
  , max_ray_depth_m(1.0)
  , img_width_px(0)
  , img_height_px(0)
  , camera_matrix( Eigen::Matrix3d::Identity() )
  {
    
  }
  
  PinholeCamRayCaster::PinholeCamRayCaster( Config config )
  : config_(config)
  , ray_directions_( boost::make_shared<RayDirectionSet>() )
  {
    computeRelRayDirections();
  }
  
  void PinholeCamRayCaster::setResolution( ResolutionSettings& res_settings )
  {
    if( config_.resolution!=res_settings )
    {
      config_.resolution = res_settings;
      computeRelRayDirections();
    }
  }
  
  void PinholeCamRayCaster::setConfig( Config config )
  {
    config_ = config;
    computeRelRayDirections();
  }
  
  PinholeCamRayCaster::RayDirection PinholeCamRayCaster::projectPixelTo3dRay( unsigned int x_px, unsigned int y_px )
  {
    RayDirection dir;
    
    dir(0) = (x_px-config_.camera_matrix(0,2))/config_.camera_matrix(0,0);
    dir(1) = (y_px-config_.camera_matrix(1,2))/config_.camera_matrix(1,1);
    dir(2) = 1.0;
    
    dir.normalize();
    
    return dir;
  }
  
  boost::shared_ptr<PinholeCamRayCaster::RaySet> PinholeCamRayCaster::getRaySet( movements::Pose& sensor_pose )
  {
    Ray ray;
    ray.origin = sensor_pose.position;
    
    boost::shared_ptr<PinholeCamRayCaster::RaySet> ray_set = boost::make_shared<PinholeCamRayCaster::RaySet>();
    
    for( RayDirection& rel_dir: *ray_directions_ )
    {
      ray.direction = sensor_pose.orientation*rel_dir;
      ray_set->push_back(ray);
    }
    
    return ray_set;
  }
  
  boost::shared_ptr<PinholeCamRayCaster::RayDirectionSet> PinholeCamRayCaster::getRayDirectionSet( movements::Pose& sensor_pose )
  {
    boost::shared_ptr<PinholeCamRayCaster::RayDirectionSet> ray_dirs = boost::make_shared<PinholeCamRayCaster::RayDirectionSet>();
    
    for( RayDirection& rel_dir: *ray_directions_ )
    {
      RayDirection abs_dir = sensor_pose.orientation*rel_dir;
      ray_dirs->push_back(abs_dir);
    }
    
    return ray_dirs;
  }
  
  boost::shared_ptr<const PinholeCamRayCaster::RayDirectionSet> PinholeCamRayCaster::getRelRayDirectionSet() const
  {
    return boost::const_pointer_cast<const RayDirectionSet>(ray_directions_);
  }
  
  void PinholeCamRayCaster::computeRelRayDirections()
  {
    ray_directions_->clear();
    
    double min_x = config_.resolution.min_x_perc*config_.img_width_px;
    double max_x = config_.resolution.max_x_perc*config_.img_width_px;
    double min_y = config_.resolution.min_y_perc*config_.img_height_px;
    double max_y = config_.resolution.max_y_perc*config_.img_height_px;
    
    
    double x_step = 1.0/config_.resolution.ray_resolution_x;
    double y_step = 1.0/config_.resolution.ray_resolution_y;
    
    for( double x = min_x; x<=max_x; x+=x_step )
    {
      for( double y = min_y; y<=max_y; y+=y_step )
      {
	RayDirection ray_dir = projectPixelTo3dRay(x,y);
	ray_directions_->push_back(ray_dir);
      }
    }
  }
  
}

}