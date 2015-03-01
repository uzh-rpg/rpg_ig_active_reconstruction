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

#include "movements/geometry_pose.h"
#include <movements/translation.h>
#include <movements/ros_movements.h>

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
 
  
  if( !planAndMove() ) // don't have to move for first position
    return true; // no calculation for current new pos, but still new positions available
  
  
  ros::Duration(1).sleep(); // sleep one second - allow robot to move
  
  return true;
}

bool YoubotReconstructionController::isCollisionFree( planning_scene_monitor::LockedPlanningSceneRO& _scene, robot_state::RobotState& _robot )
{
  bool colliding = _scene->isStateColliding( _robot );
    
  return !colliding;
}
int test = 0;
bool YoubotReconstructionController::planAndMove()
{
  std::string end_effector_name;
  end_effector_name = robot_->getEndEffector();
  std::cout<<std::endl<<"end effector name: "<<end_effector_name<<std::endl;
  
  geometry_msgs::PoseStamped current_end_effector_pose = robot_->getCurrentPose();
  std::cout<<std::endl<<current_end_effector_pose<<std::endl;
  
  using namespace std;
  cout<<endl<<"goal position tolerance is: "<<robot_->getGoalPositionTolerance();
  cout<<endl<<"goal orientation tolerance is: "<<robot_->getGoalOrientationTolerance();
  cout<<endl;
  
  robot_->setGoalPositionTolerance(0.001);
  robot_->setGoalOrientationTolerance(0.001);
  
  ros::Duration(2).sleep();
  geometry_msgs::Pose target_0, target_1;
  
  target_0.position.x = 0.126;
  target_0.position.y = -0.01;
  target_0.position.z = 0.477;
  target_0.orientation.x = 0.019;
  target_0.orientation.y = 0.109;
  target_0.orientation.z = -0.99378;
  target_0.orientation.w = 0.0109;
  
  target_1.position.x = 0.126;
  target_1.position.y = -0.01;
  target_1.position.z = 0.377;
  target_1.orientation.x = 0.019;
  target_1.orientation.y = 0.109;
  target_1.orientation.z = -0.99378;
  target_1.orientation.w = 0.0109;
  
  movements::GeometryPose base_pose = movements::ROSToMovements(target_0);
  
  std::vector<geometry_msgs::Pose> cartesian_path;
  if( test == 0 )
  {
    robot_->setPoseTarget(target_0);
    test = 1;
  }
  else
  {
    robot_->setPoseTarget(target_1);
    test = 0;
  }
  
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