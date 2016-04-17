/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of movements, a library for representations and calculations of movements in space,

movements is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
movements is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with movements. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "movements/kinematic_movement_description.h"
#include "movements/geometry_pose.h"
#include "movements/translation.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace movements
{
  
class InOutSpiral:public KinematicMovementDescription::KinematicMovementDescriptionInstance
{  
public:
  
  enum Plane{ XYPlane, YZPlane, ZXPlane, YXPlane, ZYPlane, XZPlane };
  
  /** Constructor for a new InOutSpiral
   * @param _orientation The spiral is being built in the _plane-plane of a coordinate system oriented as described by the quaternion, rotation starts at the "first" axis (see _plane description). It thus represents the rotation R_PS that transforms a point oriented as v_S in S into its corresponding point v_P = R_PS*v_S in a coordinate oriented like P
   * @param _max_radius maximal size of the spiral. If it is reached the spiral will shrink again until its radius equals zero, then it will restart growing, and so on
   * @param _angle_speed speed by which the spiral turns [rad/s]
   * @param _radial_speed speed by which the spiral grows outward [m/s]
   * @param _plane used to set the plane in which the spiral shall be created, relative to the local coordinate frame (_orientation). Options are XYPlane, YZPlane, ZXPlane, YXPlane, ZYPlane and XZPlane. For XY, YZ and ZX the spiral rotates according to the right hand rule around the normal axis, for YX, ZY and XZ it rotates according to the left hand rule
   */
  InOutSpiral( Eigen::Quaterniond _orientation, double _max_radius, double _angle_speed, double _radial_speed, Plane _plane=XYPlane );
  
  /** directly returns a kinematic movement description containing an InOutSpiral, see the constructor for a description of the parameters */
  static KinematicMovementDescription create(  Eigen::Quaterniond _orientation, double _max_radius, double _angle_speed, double _radial_speed, Plane _plane=XYPlane );
  
  virtual std::string type();
  
  virtual RelativeMovement operator()( double _time );
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  Eigen::Quaterniond orientation_;
  double max_radius_;
  double angle_speed_;
  double radial_speed_;
  
  Plane plane_to_use_; // the plane in which the spiral is being created
  
  double half_time_; /// time needed to reach outer bound
  
  /** calculates the radius at time _time */
  double getRadius( double _time );
};

}