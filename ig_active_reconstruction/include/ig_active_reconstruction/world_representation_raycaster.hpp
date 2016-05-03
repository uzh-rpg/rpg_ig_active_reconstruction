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

#include <Eigen/Core>
#include <movements/core>

#include <vector>
#include <memory>

namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
  /*! Abstract base class defining convenient tools for raycasting.
  */
  class RayCaster
  {
  public:
    
    typedef Eigen::Vector3d RayOrigin;
    typedef Eigen::Vector3d RayDirection;
    
    /*! Defines a ray.
     */
    struct Ray
    {
      RayOrigin origin;
      RayDirection direction;
    };
    
    typedef std::vector<Ray> RaySet;
    typedef std::vector<RayOrigin> RayOriginSet;
    typedef std::vector<RayDirection> RayDirectionSet;
    
  public:
    virtual ~RayCaster(){};
    
    /*! Returns a set of rays cast from sensor_pose with the current configuration
     * @param sensor_pose Position from which rays are cast.
     * @return Pointer to a set of rays.
     */
    virtual std::shared_ptr<RaySet> getRaySet( movements::Pose& sensor_pose )=0;
    
    /*! Returns the set of ray directions as the would be cast from the given sensor_pose with the current configuration.
     * @param sensor_pose Position from which rays are cast.
     * @return Pointer to a set of ray directions.
     */
    virtual std::shared_ptr<RayDirectionSet> getRayDirectionSet( movements::Pose& sensor_pose )=0;
    
    /*! Returns the set of ray directions as cast for the current configuration, relative to the sensor.
     */
    virtual std::shared_ptr<const RayDirectionSet> getRelRayDirectionSet() const=0;
  };

  
}

}