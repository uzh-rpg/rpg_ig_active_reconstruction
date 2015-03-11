/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of dense_reconstruction, a ROS package for...well,

dense_reconstruction is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
dense_reconstruction is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with dense_reconstruction. If not, see <http://www.gnu.org/licenses/>.
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

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ActionClient;

namespace dense_reconstruction
{
  
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
  
  /** attempts to get a new end effector pose from tf (transform from end effector frame to robot base)
   * @param _max_wait_time the maximal time to wait for a new transformation to be available
   * @return empty Pose() if no complete tf tree was published for the transformation in the given time
   */
  movements::Pose getEndEffectorPoseFromTF( ros::Duration _max_wait_time= ros::Duration(5.0) );
  
  /** loads pose of link _link relative to the planning frame (using tf, not moveit)
   * That is transform to transforms entities in _link frame to the planning frame
   */
  movements::Pose getCurrentLinkPose( std::string _link );
  
  /** plans and executes a scanning movement from the current position of arm_link_4
   * @param _max_dropoff see filteredPlanFromMovementsPath
   * @return true if successful, false if not
   */
  bool makeScan( double _max_dropoff=0.2 );
  
  /** moves the base to the target position on a circular (but with varying radius) trajectory around the given _center
   * @return true if successful, false if not
   */
  bool moveBaseCircularlyTo( Eigen::Vector2d _target_position, Eigen::Vector2d _center );
  
  /** commands the base to move to a specific position, doesn't check for success though!
   */
  void moveBaseTo( double _x, double _y, double _theta );
  
  /** executes a trajectory on the base (positions only atm)
   * @param _path the path to follow
   * @param _dt time to pass between sending commands [s]
   */
  bool executeMovementsTrajectoryOnBase( movements::PoseVector& _path, double _dt );
  
  /** attempts to create a cartesian path following the given poses given some constraints
   * @return true if planning and execution were successful
   */
  bool executeMovementsPath( movements::PoseVector& _path, moveit_msgs::Constraints* _constraints=nullptr, double _max_dropoff=0.2 );
  
  /** Calls computeCartesianPath(...) to build a moveit plan for the movements path for the end effector. Path constraints are cleared afterwards, this will affect all constraints set for the robot_ object!
   * @param _waypoints path for the end effector
   * @param _plan returned path
   * @param _planning_attempts number of planning attempts to be taken if planning fails before giving up
   * @param _path_constraints path constraints to use during planning
   * @return true if plan could be calculated, false if not
   */
  bool planFromMovementsPath( movements::PoseVector& _waypoints, moveit::planning_interface::MoveGroup::Plan& _plan, moveit_msgs::Constraints* _path_constraints=nullptr, int _planning_attempts=3 );
  
  /** Calls computeCartesianPath(...) to build a moveit plan for the movements path for the end effector. Path constraints are cleared afterwards, this will affect all constraints set for the robot_ object! If planning fails for _planning_attempts times, the function tries to localize path points that are a problem and removes them. The maximal number or percentage of points that can be removed before the function returns failure can be specified in _max_dropoff: currently only works if the resolution (max distance between points) of the path passed is less than eef_step used in the cartesian path calculation inside the function (currently 0.1m)
   * @param _waypoints path for the end effector
   * @param _plan returned path
   * @param _planning_attempts number of planning attempts to be taken if planning fails before giving up
   * @param _path_constraints path constraints to use during planning
   * @param _max_dropoff If less than 1: Represents the percentage of the maximal number of points that may be dropped to find a valid cartesian path, if equal or higher than 1 it represents the absolute number of points that may be dropped, if _max_dropoff<=0 no filter stage is run
   * @return true if plan could be calculated, false if not
   */
  bool filteredPlanFromMovementsPath( movements::PoseVector& _waypoints, moveit::planning_interface::MoveGroup::Plan& _plan, moveit_msgs::Constraints* _path_constraints=nullptr, int _planning_attempts=3, double _max_dropoff = 0.2 );
  
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
  
  /** Attempts to set up the tf structure in order to combine the svo and robot trees. The origin is currently equal to the odom origin, a fixed transform is being setup between the svo frame and the dr_origin frame by using the transform between one SVO pose at startup and the robot tree, waits until SVO cam_pos is available on /tf */
  void initializeTF();
  
private:
  ros::NodeHandle* ros_node_;
  ros::ServiceClient eye_client_;
  ros::ServiceClient hand_client_;
  ActionClient base_trajectory_sender_;
  
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

}