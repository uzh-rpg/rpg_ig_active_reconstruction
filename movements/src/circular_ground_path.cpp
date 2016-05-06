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

#include "movements/circular_ground_path.h"
#include <angles/angles.h>

#include <stdexcept>

namespace movements
{

CircularGroundPath::CircularGroundPath( Eigen::Vector3d _start_point, Eigen::Vector3d _target_point, double _angular_speed, MovementDirection _direction ):
  start_point_(_start_point.x(),_start_point.y()),
  end_point_(_target_point.x(),_target_point.y()),
  angular_speed_(_angular_speed),
  direction_(_direction)
{
  
}

CircularGroundPath::CircularGroundPath( movements::Pose _start_point, movements::Pose _target_point, double _angular_speed, MovementDirection _direction ):
  start_point_(_start_point.position.x(),_start_point.position.y()),
  end_point_(_target_point.position.x(),_target_point.position.y()),
  angular_speed_(_angular_speed),
  direction_(_direction)
{
  
}


KinematicMovementDescription CircularGroundPath::create( Eigen::Vector3d _start_point, Eigen::Vector3d _target_point, double _angular_speed, MovementDirection _direction )
{
  return KinematicMovementDescription( new CircularGroundPath( _start_point, _target_point, _angular_speed, _direction ) );
}


KinematicMovementDescription CircularGroundPath::create( movements::Pose _start_point, movements::Pose _target_point, double _angular_speed, MovementDirection _direction )
{
  return KinematicMovementDescription( new CircularGroundPath( _start_point, _target_point, _angular_speed, _direction ) );
}

double CircularGroundPath::totalAngle( movements::Pose& _center )
{
  if( _center.position.x()==start_point_.x() && _center.position.y()==start_point_.y() ||
      _center.position.x()==end_point_.x() && _center.position.y()==end_point_.y()
  )
  {
    throw std::invalid_argument("CircularGroundPath::totalAngle:: Invalid argument: The path center provided has the same projection as either the start- or the endpoint of the circular ground path, which is not allowed.");
  }
  
  Eigen::Vector2d center( _center.position.x(), _center.position.y() );
  Eigen::Vector2d start_local = RelativePositionCalculator::localCoordinates( _center.position, start_point_ );
  Eigen::Vector2d end_local = RelativePositionCalculator::localCoordinates( _center.position, end_point_ );
  
  double phi_start_local = acos(start_local.x()/start_local.norm());
  if( start_local.y()<0 )
    phi_start_local = 2*M_PI - phi_start_local;
  double phi_end_local = acos(end_local.x()/end_local.norm());
  if( end_local.y()<0 )
    phi_end_local = 2*M_PI - phi_end_local;
  
  MovementDirection direction_to_choose = direction_;
  
  double phi_end_localrot = angles::normalize_angle_positive( phi_end_local-phi_start_local );
  if( direction_==SHORTEST )
  {
    if( phi_end_localrot > M_PI )
      direction_to_choose = CLOCKWISE;
    else
      direction_to_choose = COUNTER_CLOCKWISE;
  }
  double total_angle_to_move;
  if( direction_to_choose == COUNTER_CLOCKWISE )
  {
    if( start_point_==end_point_ )
      total_angle_to_move = 2*M_PI;
    else
      total_angle_to_move = phi_end_localrot;
  }
  else // CLOCKWISE
    total_angle_to_move = 2*M_PI - phi_end_localrot;
  
  return total_angle_to_move;
}

std::string CircularGroundPath::type()
{
  return "CircularGroundPath";
}

RelativeMovement CircularGroundPath::operator()( double _time )
{
  return RelativeMovement( new RelativePositionCalculator( start_point_, end_point_, angular_speed_, direction_, _time ) );
}


CircularGroundPath::RelativePositionCalculator::RelativePositionCalculator( Eigen::Vector2d _start_point, Eigen::Vector2d _target_point, double _angular_speed, MovementDirection _direction, double _time ):
  start_point_(_start_point),
  end_point_(_target_point),
  angular_speed_(fabs(_angular_speed)),
  direction_(_direction),
  time_(_time)
{
  
}

movements::Pose CircularGroundPath::RelativePositionCalculator::applyToBasePose( movements::Pose const& _path_center )
{
  if( _path_center.position.x()==start_point_.x() && _path_center.position.y()==start_point_.y() ||
      _path_center.position.x()==end_point_.x() && _path_center.position.y()==end_point_.y()
  )
  {
    throw std::invalid_argument("CircularGroundPath::RelativePositionCalculator::applyToBasePose:: Invalid argument: The path center provided has the same projection as either the start- or the endpoint of the circular ground path, which is not allowed.");
  }
  
  Eigen::Vector2d center( _path_center.position.x(), _path_center.position.y() );
  Eigen::Vector2d start_local = localCoordinates( _path_center.position, start_point_ );
  Eigen::Vector2d end_local = localCoordinates( _path_center.position, end_point_ );
  
  double phi_start_local = acos(start_local.x()/start_local.norm());
  if( start_local.y()<0 )
    phi_start_local = 2*M_PI - phi_start_local;
  double phi_end_local = acos(end_local.x()/end_local.norm());
  if( end_local.y()<0 )
    phi_end_local = 2*M_PI - phi_end_local;
  
  MovementDirection direction_to_choose = direction_;
  
  double phi_end_localrot = angles::normalize_angle_positive( phi_end_local-phi_start_local );
  if( direction_==SHORTEST )
  {
    if( phi_end_localrot > M_PI )
      direction_to_choose = CLOCKWISE;
    else
      direction_to_choose = COUNTER_CLOCKWISE;
  }
  double total_angle_to_move;
  if( direction_to_choose == COUNTER_CLOCKWISE )
  {
    if( start_point_==end_point_ )
      total_angle_to_move = 2*M_PI;
    else
      total_angle_to_move = phi_end_localrot;
  }
  else // CLOCKWISE
    total_angle_to_move = 2*M_PI - phi_end_localrot;
  
  Eigen::Vector2d center2end = end_point_-center;
  Eigen::Vector2d center2start = start_point_-center;
  double radial_distance_to_move = center2end.norm()-center2start.norm();
  
  double total_time_for_execution = total_angle_to_move/angular_speed_;
  double radial_speed = radial_distance_to_move/total_time_for_execution;
  
  Eigen::Vector2d out_point_local;
  
  if( time_>=total_time_for_execution )
  {
    out_point_local = end_local;
  }
  else if( time_<=0 ) // cases could be sped up by doing this test at the beginning, however this isn't considered a regular case and it would make the code look messy
  {
    out_point_local = start_local;
  }
  else // point on path
  {
    double out_angle;
    if( direction_to_choose == COUNTER_CLOCKWISE )
    {
      out_angle = angular_speed_*time_ + phi_start_local;
    }
    else // CLOCKWISE
    {
      out_angle = -angular_speed_*time_ + phi_start_local;
    }
    
    out_point_local = ( start_local.norm()+radial_speed*time_ ) * Eigen::Vector2d( cos(out_angle), sin(out_angle) );
  }
  
  Eigen::Vector3d out_point( out_point_local.x()+_path_center.position.x(), out_point_local.y()+_path_center.position.y(), _path_center.position.z() );
  Eigen::Vector3d out2center =  _path_center.position-out_point;
  double orientation_angle = acos( out2center.x()/out2center.norm() );
  if( out2center.y()<0 )
    orientation_angle = 2*M_PI-orientation_angle;
  double orientation_angle_half = orientation_angle/2;
  Eigen::Quaterniond out_orientation( cos(orientation_angle_half),0,0,sin(orientation_angle_half) ); //Quaternion(w,x,y,z)
  
  return movements::Pose(out_point,out_orientation);
}

std::string CircularGroundPath::RelativePositionCalculator::type()
{
  return "CircularGroundPath::RelativePositionCalculator";
}

Eigen::Vector2d CircularGroundPath::RelativePositionCalculator::localCoordinates( Eigen::Vector3d const& _center, Eigen::Vector2d global_coordinates )
{
  Eigen::Vector2d local;
  local.x() = global_coordinates.x() - _center.x();
  local.y() = global_coordinates.y() - _center.y();
  return local;
}

}