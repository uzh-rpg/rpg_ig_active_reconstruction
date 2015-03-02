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
#include "utils/ros_eigen.h"


YoubotReconstructionController::YoubotReconstructionController( ros::NodeHandle* _n ):
  ros_node_(_n),
  tf_listener_(*_n)
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

movements::Pose YoubotReconstructionController::getEndEffectorPoseFromTF( ros::Duration _max_wait_time )
{
  bool new_tf_available = tf_listener_.waitForTransform( "/base_footprint","/camera", ros::Time::now(), ros::Duration(3.0) );
  
  if( !new_tf_available )
    return movements::Pose();
  
  tf::StampedTransform end_effector_pose_tf;
  geometry_msgs::TransformStamped end_effector_pose_ros;
  
  try{
    tf_listener_.lookupTransform("/base_footprint", "/camera", ros::Time(0), end_effector_pose_tf);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return movements::Pose();
  }
  
  tf::transformStampedTFToMsg( end_effector_pose_tf, end_effector_pose_ros );
  
  Eigen::Quaterniond rotation = st_is::geometryToEigen( end_effector_pose_ros.transform.rotation );
  Eigen::Vector3d translation = st_is::geometryToEigen( end_effector_pose_ros.transform.translation );
  
  return movements::Pose( translation, rotation );
}

bool YoubotReconstructionController::isCollisionFree( planning_scene_monitor::LockedPlanningSceneRO& _scene, robot_state::RobotState& _robot )
{
  bool colliding = _scene->isStateColliding( _robot );
    
  return !colliding;
}

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
  
  robot_->setGoalPositionTolerance(0.001);
  robot_->setGoalOrientationTolerance(0.001);


  // using direct pose targets
  /*
  if( test == 0 )
  {
    robot_->setPoseTarget( movements::toROS(base_pose) );
    test = 1;
  }
  else
  {
    robot_->setPoseTarget( movements::toROS( base_pose+md(0.1) ) );
    test = 0;
  }
  */
  // using cartesian paths
  cout<<endl<<"Enter 1 to execute the relative trajactory, 0 to move to the base position."<<endl;
  int input;
  cin>>input;
  if( input == 0 )
  {
    cout<<endl<<"Moving to start position"<<endl;
    robot_->setPoseTarget( target_0 );
    
    planning_scene_monitor::LockedPlanningSceneRO current_scene( scene_ );
    robot_state::RobotState current_robot_state = current_scene->getCurrentState();
    //robot_->setStartState(current_robot_state);
    
    //robot_->setStartStateToCurrentState();
    
    //planning_scene_monitor::LockedPlanningSceneRO current_scene( scene_ );
    //robot_state::RobotState current_robot_state = current_scene->getCurrentState();
    //robot_->setStartState(current_robot_state);
    
    double joint_tolerance = robot_->getGoalJointTolerance();
    double velocity_tolerance = 0.0001;
    
    
    ros::AsyncSpinner spinner(1);
    scene_->unlockSceneRead();
    
    spinner.start();
    // plan and execute a path to the target state
    cout<<endl<<"well i get here.."<<endl;
    bool success;
    do{success = robot_->move();}while(!success);
    cout<<endl<<"moveit says that the motion execution function has finished with success="<<success<<"."<<endl;
    spinner.stop();
    scene_->lockSceneRead();
    
    //if( !success ) return false;
    
    
  }
  else
  {
    cout<<endl<<"Executing motion plan"<<endl;
    
    //robot_->setStartStateToCurrentState();
    
    //planning_scene_monitor::LockedPlanningSceneRO current_scene( scene_ );
    //robot_state::RobotState current_robot_state = current_scene->getCurrentState();
    //robot_->setStartState(current_robot_state);
    
    geometry_msgs::Pose current_pose = robot_->getCurrentPose().pose;
    cout<<endl<<"The reference frame is: "<<robot_->getPoseReferenceFrame();
    cout<<endl<<"The end effector frame is: "<<robot_->getEndEffector();
    cout<<endl<<"The current pose appears to be:"<<endl<<current_pose<<endl;
    
    movements::Pose base_pose = getEndEffectorPoseFromTF();//movements::fromROS(current_pose);
    movements::RelativeMovement z_down = movements::Translation::create(0,0,-0.1);
    movements::KinMove md = movements::Linear::create(0,0,-1,1); // moving downwards with 1 m/s
    
    std::vector<movements::Pose> m_waypoints = md.path( base_pose, 0, 0.1, 0.05 );
    std::vector<geometry_msgs::Pose> waypoints = toROS(m_waypoints);
    
    moveit_msgs::RobotTrajectory trajectory;
    moveit::planning_interface::MoveGroup::Plan plan;
    
    double success_ratio = 0;
    int count=0;
    while(success_ratio!=1)
    {
      count++;
      success_ratio=robot_->computeCartesianPath( waypoints, 1,0 /*0.2 = ~10 degree max dist in config space, 0 disables it*/, trajectory );
      ros::Duration(0.01).sleep();
      cout<<endl<<"Trying to solve problem...";
      //break;
      if( count>10 ) break;
    }
    cout<<endl<<"The planning success ratio is "<<success_ratio<<"%.";
    cout<<endl<<"calculated path has size :"<<endl<<waypoints.size()<<".";
    cout<<endl<<"The poses in the calculated path are:";
    BOOST_FOREACH(auto pose, waypoints)
    {
      cout<<pose<<endl;
    }
    cout<<endl<<"The computed trajectory is:"<<endl<<trajectory<<endl<<endl;
    
    plan.trajectory_ = trajectory;
    plan.planning_time_ = 0.1;
  
    
    ros::AsyncSpinner spinner(1);    
    //scene_->unlockSceneRead();    
    spinner.start();
    if(success_ratio==1) robot_->execute(plan);
    else cout<<endl<<"Failed to create complete trajectory."<<endl;
    spinner.stop();
    //scene_->lockSceneRead();
        
  }
  
  
  
  
    
  ros::Duration(5).sleep();
  ros::Duration wait_time(0,10000000); // 10 ms
  
  
  bool finished = false;
  
  
  return true;
}