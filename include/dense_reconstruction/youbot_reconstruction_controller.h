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

#include <sensor_msgs/CameraInfo.h>

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
  
private:
  ros::NodeHandle* ros_node_;
  ros::ServiceClient eye_client_;
  ros::ServiceClient hand_client_;
  
  boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> scene_;
  boost::shared_ptr<moveit::planning_interface::MoveGroup> robot_;
    
  /// plans and executes a plan to the currently loaded target - blocks until completion
  /** completion means that the robot state is closer to the target than set in the tolerance and its velocity is approximately zero in all joint
   * @return true if movement was executed successfully
   */
  bool planAndMove();
  
  /// NOT WORKING IN THIS FILE; JUST TO LOOK AT AND ALTER calculates the calibration pattern coordinates in camera frame coordinates given a robot state
  /**
   * @param _robot robot state
   * @return camera pose:   The transformation from calibration pattern world coordinates (O) to eye coordinates (E): the rotation R_EO and the translation vector E_t_EO,  Therefore, a transformation O->E would be carried out through: x_E = R_EO*x_O + E_t_EO
   */
  geometry_msgs::Pose getCameraWorldPose( robot_state::RobotState& _robot );
  
  /// returns whether MoveIt believes the robot state represented in _robot to be free of collisions or not given the current scene (but without the calibration pattern)
  /**
   * @param _robot robot state to check
   * @param _scene the current scene
   */
  bool isCollisionFree( planning_scene_monitor::LockedPlanningSceneRO& _scene, robot_state::RobotState& _robot );
  
  
};