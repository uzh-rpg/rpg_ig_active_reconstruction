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

#include <movements/core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace movements
{

/** calculates circular path with varying radius from a start point to an end point in the xy-plane. Z-coordinates are not considered, when absolute points are created the have the same z-value as the point of reference (relative point). The relative point will be used as center. The center may not be equal in x and y coordinates to start or end point. The trajectory does always yield the end point if t>total_time, negative time yields the start point. Output orientations are such that the x-axis points toward the point of reference*/
class CircularGroundPath: public KinematicMovementDescription::KinematicMovementDescriptionInstance
{
public:
  enum MovementDirection{ SHORTEST, COUNTER_CLOCKWISE, CLOCKWISE };
  class RelativePositionCalculator; /// one point on circular ground path
  
  /** constructor for CircularGroundPath
   * @param _start_point start point of the movement
   * @param _target_point end point of the movement
   * @param _angular_speed angular speed [rad/s]
   * @param _direction in which direction to turn (SHORTEST, COUNTER_CLOCKWISE, CLOCKWISE), default is SHORTEST
   */
  CircularGroundPath( Eigen::Vector3d _start_point, Eigen::Vector3d _target_point, double _angular_speed, MovementDirection _direction = SHORTEST );
  /** constructor for CircularGroundPath
   * @param _start_point the positions vector inside the pose will be used directly as start point of the movement
   * @param _target_point the positions vector inside the pose will be used directly as end point of the movement
   * @param _angular_speed angular speed [rad/s]
   * @param _direction in which direction to turn (SHORTEST, COUNTER_CLOCKWISE, CLOCKWISE), default is SHORTEST
   */
  CircularGroundPath( movements::Pose _start_point, movements::Pose _target_point, double _angular_speed, MovementDirection _direction = SHORTEST );
  
  /** directly returns a KinematicMovementDescription containing a CircularGroundPath movement, see constructor for description of the parameters */
  static KinematicMovementDescription create( Eigen::Vector3d _start_point, Eigen::Vector3d _target_point, double _angular_speed, MovementDirection _direction = SHORTEST );
  /** directly returns a KinematicMovementDescription containing a CircularGroundPath movement, see constructor for description of the parameters */
  static KinematicMovementDescription create( movements::Pose _start_point, movements::Pose _target_point, double _angular_speed, MovementDirection _direction = SHORTEST );
  
  /** returns the total angle to move from start to end position for a given center */
  double totalAngle( movements::Pose& _center );
  
  virtual std::string type();
  
  virtual RelativeMovement operator()( double _time );
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  Eigen::Vector2d start_point_;
  Eigen::Vector2d end_point_;
  double angular_speed_;
  MovementDirection direction_;
};

class CircularGroundPath::RelativePositionCalculator: public RelativeMovement::RelativeMovementInstance
{
public:
  RelativePositionCalculator( Eigen::Vector2d _start_point, Eigen::Vector2d _target_point, double _angular_speed, MovementDirection _direction, double _time );
  
  virtual std::string type();
  
  /** returns the Pose represented by the CircularGroundPath::RelativePositionCalculator if _path_center is used as center of the circular path
   * @throws invalid_argument if the path center equals the start or end point in x and y coordinates (same projection on xy-plane)
   */
  virtual movements::Pose applyToBasePose( movements::Pose const& _path_center );
  
  /** calculates local 2d coordinates relative to the x/y coordinates of _center */
  static Eigen::Vector2d localCoordinates( Eigen::Vector3d const& _center, Eigen::Vector2d global_coordinates );
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  Eigen::Vector2d start_point_; // [m]
  Eigen::Vector2d end_point_;
  double angular_speed_; // [rad/s]
  MovementDirection direction_;
  double time_; //[s]
  
};

}