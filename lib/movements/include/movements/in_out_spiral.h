/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
* 
in_out_spiral is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
in_out_spiral is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with in_out_spiral. If not, see <http://www.gnu.org/licenses/>.
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
  /** Constructor for a new InOutSpiral
   * @param _orientation The spiral is being built in the x-y-plane of a coordinate system oriented as described by the quaternion, rotation starts at the x-axis. It thus represents the rotation R_PS that transforms a point oriented as v_S in S into its corresponding point v_P = R_PS*v_S in a coordinate oriented like P
   * @param _max_radius maximal size of the spiral. If it is reached the spiral will shrink again until its radius equals zero, then it will restart growing, and so on
   * @param _angle_speed speed by which the spiral turns [rad/s]
   * @param _radial_speed speed by which the spiral grows outward [m/s]
   */
  InOutSpiral( Eigen::Quaterniond _orientation, double _max_radius, double _angle_speed, double _radial_speed );
  
  std::string type();
  
  RelativeMovement operator()( double _time );
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  Eigen::Quaterniond orientation_;
  double max_radius_;
  double angle_speed_;
  double radial_speed_;
  
  double half_time_; /// time needed to reach outer bound
  
  /** calculates the radius at time _time */
  double getRadius( double _time );
};

}