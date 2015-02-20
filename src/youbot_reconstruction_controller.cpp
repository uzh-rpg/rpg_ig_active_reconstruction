/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of hand_eye_calibration, a ROS package for hand eye calibration,

hand_eye_calibration is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
hand_eye_calibration is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with hand_eye_calibration. If not, see <http://www.gnu.org/licenses/>.
*/


#include <string>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <boost/foreach.hpp>
#include "dense_reconstruction/youbot_reconstruction_controller.h"

YoubotReconstructionController::YoubotReconstructionController( ros::NodeHandle* _n ):
  ros_node_(_n)
{  
  std::string moveit_group_name = "arm";
  
  
  scene_ = boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>( new planning_scene_monitor::PlanningSceneMonitor("robot_description") );
  scene_->startStateMonitor();
  scene_->startSceneMonitor();
  scene_->startWorldGeometryMonitor();
  
  robot_ = boost::shared_ptr<moveit::planning_interface::MoveGroup>( new moveit::planning_interface::MoveGroup(moveit_group_name) );
    
}

bool YoubotReconstructionController::runSingleIteration()
{
  ROS_INFO("Calculate next position...");
  bool found_new_pos = true;//calculateNextJointPosition();
  if( !found_new_pos )
    return false;
  //else
  //  setTargetToNewPosition();
  
  if( !planAndMove() ) // don't have to move for first position
    return true; // no calculation for current new pos, but still new positions available
  
  
  ros::Duration(1).sleep(); // sleep one second - allow robot to move
  
  return true;
}

geometry_msgs::Pose YoubotReconstructionController::getCameraWorldPose( robot_state::RobotState& _robot )
{
  /* calculation of next camera pose t_EP
   * **********************************
   * space indices used:
   * - E: eye space
   * - P: calibration pattern space
   * - B: robot base space
   * - H: hand space
   * - R: space of last actuated robot link
   * - G: planning frame (model frame) of the moveit robot object
   */
  /*TransformationEstimator::EstimationData current_hec_estimate("daniilidis_1998", Eigen::Quaterniond(0.707, 0, 0.707, 0), Eigen::Vector3d(0,0,-0.04) );// = estimators_.front()->estimate();
  st_is::CoordinateTransformation t_EH( current_hec_estimate.rot_EH(), current_hec_estimate.E_trans_EH() );
  
  st_is::CoordinateTransformation t_BP;// = estimators_.front()->getCalibrationPatternPoseEstimate(current_hec_estimate);
    
  Eigen::Matrix<double,4,4> m_HG = _robot.getFrameTransform( robot_hand_frame_ ).matrix();
  Eigen::Quaterniond quat_HG( m_HG.topLeftCorner<3,3>() );
  Eigen::Vector3d H_t_HG = m_HG.topRightCorner<3,1>();
  st_is::CoordinateTransformation t_HG( quat_HG, H_t_HG );
  
  Eigen::Matrix<double,4,4> m_BG = _robot.getFrameTransform( robot_base_frame_ ).matrix();
  Eigen::Quaterniond quat_BG( m_BG.topLeftCorner<3,3>() );
  Eigen::Vector3d B_t_BG = m_BG.topRightCorner<3,1>();
  st_is::CoordinateTransformation t_BG( quat_BG, B_t_BG );
  
  st_is::CoordinateTransformation t_GB = t_BG.inv();  
  st_is::CoordinateTransformation t_HB = t_HG*t_GB;
  st_is::CoordinateTransformation t_EB = t_EH*t_HB;
  
  st_is::CoordinateTransformation t_EP = t_EB*t_BP;
  geometry_msgs::Pose camera_pose;
  camera_pose.orientation = st_is::eigenToGeometry(t_EP.rotation);
  camera_pose.position = st_is::eigenToGeometry(t_EP.translation);
  */
  geometry_msgs::Pose camera_pose;
  return camera_pose;
}

bool YoubotReconstructionController::isCollisionFree( planning_scene_monitor::LockedPlanningSceneRO& _scene, robot_state::RobotState& _robot )
{
  bool colliding = _scene->isStateColliding( _robot );
    
  return !colliding;
}

bool YoubotReconstructionController::planAndMove()
{
  
  std::vector<double> target_state_position;
  
  std::vector<std::string> joint_names = robot_->getJoints();
  
  // move() and execute() never unblock thus this target reaching function is used
  robot_state::RobotState target_robot_state = robot_->getJointValueTarget();
  
    
  planning_scene_monitor::LockedPlanningSceneRO current_scene( scene_ );
  robot_state::RobotState current_robot_state = current_scene->getCurrentState();
  robot_->setStartState(current_robot_state);
  
  double joint_tolerance = robot_->getGoalJointTolerance();
  double velocity_tolerance = 0.0001;
  
  
  ros::AsyncSpinner spinner(1);
  
  scene_->unlockSceneRead();
  
  spinner.start();
  // plan and execute a path to the target state
  bool success = robot_->move();
  spinner.stop();
  scene_->lockSceneRead();
  
  if( !success ) return false;
    
  ros::Duration wait_time(0,10000000); // 10 ms
  
  
  bool finished = false;
  
  
  return true;
}