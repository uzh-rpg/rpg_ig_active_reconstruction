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

#pragma once

#include "ros/ros.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Core>

#include <tf/transform_listener.h>

#include <sensor_msgs/CameraInfo.h>

#include <movements/core>
#include <movements/ros_movements.h>
#include <movements/translation.h>
#include <movements/linear_movement.h>
#include <movements/in_out_spiral.h>

/// class that autonomously extracts hand-eye pose correspondences in a robotic setup and estimates the hand-eye-calibration from it
class YoubotReconstructionController
{
public:
  
  /// constructor
  /** Initializes the autonomous hand eye calibrator by reading all parameters from the parameter server
   * @param _n handle of the node the calibrator runs in
   * @throws ROS_FATAL if not all necessary parameters are given and shuts down the node
   */
  YoubotReconstructionController( ros::NodeHandle* _n );
  
  /// runs one step of the autonomous calibration process
  /** The method iterates through the joint space and estimates the hand-eye transformation along the way.
  * Based on the hand-eye transformation it also estimates the position of the calibration target, e.g.
  * the checkerboard in order to skip joint positions where the target is expected not to be visible. The
  * current estimate is printed to the console. At each run another joint position is assumed.
  * @return false if all positions were covered
  */
  bool runSingleIteration();
  
  /** attempts to get a new end effector pose from tf
   * @param _max_wait_time the maximal time to wait for a new transformation to be available
   * @return empty Pose() if no complete tf tree was published for the transformation in the given time
   */
  movements::Pose getEndEffectorPoseFromTF( ros::Duration _max_wait_time= ros::Duration(5.0) );
  
  /** Calls computeCartesianPath(...) to build a moveit plan for the movements path for the end effector. Path constraints are cleared afterwards, this will affect all constraints set for the robot_ object!
   * @param _waypoints path for the end effector
   * @param _plan returned path
   * @param _planning_attempts number of planning attempts to be taken if planning fails before giving up
   * @param _path_constraints path constraints to use during planning
   * @return true if plan could be calculated, false if not
   */
  bool planFromMovementsPath( std::vector<movements::Pose>& _waypoints, moveit::planning_interface::MoveGroup::Plan& _plan, moveit_msgs::Constraints* _path_constraints=nullptr, int _planning_attempts=3 );
  
  /** creates a static orientation constraint for the end effector based on the pose _base_pose that can be added to a moveit_msgs::Constraints by pushing it onto its orientation_constraints vector 
   * @param _base_pose pose whose orientation will be used to construct the constraint
   * @param _weight weight that is set for the constraint (importance if several constraints were to contradict each other)
   * @param _x_axis_tolerance tolerance for the x axis
   * @param _y_axis_tolerance tolerance for the y axis
   * @param _z_axis_tolerance tolerance for the z axis
   */
  moveit_msgs::OrientationConstraint getFixedEEFLinkOrientationConstraint( movements::Pose& _base_pose, int _weight=100, double _x_axis_tolerance=0.05, double _y_axis_tolerance=0.05, double _z_axis_tolerance=0.05 );
  
  /** sets the end effector planning frame
   */
  void setEndEffectorPlanningFrame( std::string _name );
  
private:
  ros::NodeHandle* ros_node_;
  ros::ServiceClient eye_client_;
  ros::ServiceClient hand_client_;
  
  std::string planning_group_; // the group for which planning is done
  std::string base_planning_frame_; /// relative base frame for end effector calculations
  std::string end_effector_planning_frame_; /// name of the frame for which the end effector pose shall be controlled
  
  boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> scene_;
  boost::shared_ptr<moveit::planning_interface::MoveGroup> robot_;
  
  tf::TransformListener tf_listener_;
    
  /// plans and executes a plan to the currently loaded target - blocks until completion
  /** completion means that the robot state is closer to the target than set in the tolerance and its velocity is approximately zero in all joint
   * @return true if movement was executed successfully
   */
  bool planAndMove();
  
  /// returns whether MoveIt believes the robot state represented in _robot to be free of collisions or not given the current scene (but without the calibration pattern)
  /**
   * @param _robot robot state to check
   * @param _scene the current scene
   */
  bool isCollisionFree( planning_scene_monitor::LockedPlanningSceneRO& _scene, robot_state::RobotState& _robot );
  
  
};