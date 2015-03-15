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


#include <string>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <boost/foreach.hpp>
#include "dense_reconstruction/youbot_planning.h"
#include "utils/ros_eigen.h"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <geometry_msgs/Pose2D.h>
#include <movements/circular_ground_path.h>
#include "dense_reconstruction/PoseSetter.h"

namespace dense_reconstruction
{

YoubotPlanner::YoubotPlanner( ros::NodeHandle* _n ):
  ros_node_(_n),
  tf_listener_(*_n),
  base_trajectory_sender_("base_controller/follow_joint_trajectory", true)
{
  planning_group_ = "arm";
  std::string remode_control_topic = "/remode/command";
  
  robot_ = boost::shared_ptr<moveit::planning_interface::MoveGroup>( new moveit::planning_interface::MoveGroup(planning_group_) );
  
  
  //setEndEffectorPlanningFrame("camera");
  base_planning_frame_="odom";
  
  scene_ = boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>( new planning_scene_monitor::PlanningSceneMonitor("robot_description") );
  scene_->startStateMonitor();
  scene_->startSceneMonitor();
  scene_->startWorldGeometryMonitor();
  
  
  // block everything until complete tf tree is available: this doesn't help
  //tf_listener_.waitForTransform( "/base_footprint","/camera", ros::Time::now(), ros::Duration(0.0) );
  
  remode_commander_ = ros_node_->advertise<std_msgs::String>( remode_control_topic, 1 );
  remode_start_.data="SET_NEXT_FRAME_IS_REFERENCE";
  remode_stopandsmooth_.data="SMOOTH_AND_STOP_UPDATING";
  
  // base movement initialization
  base_move_angle_ = 0.1*6.283185307; // [rad] one tenth of a circle
  base_radial_speed_ = 0.1*6.283185307; // [rad/s]
  base_movement_center_ = movements::Pose( Eigen::Vector3d(1,0,0), Eigen::Quaterniond() );
}

std::string YoubotPlanner::initializePlanningFrame()
{
  // try contacting action server
  ROS_INFO_STREAM("YoubotPlanner::YoubotPlanner::trying to contact trajectory server on "<<"base_controller/follow_joint_trajectory"<<" topic...");
  base_trajectory_sender_.waitForServer();
  ROS_INFO_STREAM("YoubotPlanner::YoubotPlanner::Successfully contacted.");
  
  ROS_INFO("YoubotPlanner::YoubotPlanner::Initializing tf structures...");
  initializeTF();
  ROS_INFO("YoubotPlanner::YoubotPlanner:: tf structures Initialized.");
  
  return "dr_origin";
}

bool YoubotPlanner::initializePlanningSpace( PlanningSpaceInitializationInfo& _info )
{
  if( _info.getSpecifics()->type()!="YoubotPlanningSpaceInfo" )
  {
    ROS_ERROR_STREAM("YoubotPlanner::initializePlanningSpace::The PlanningSpaceInitializationInfo is of type '"<<_info.getSpecifics()->type()<<"'. It should be of type 'YoubotPlanningSpaceInfo'. Initialization is not possible.");
    return false;
  }
  boost::shared_ptr<SpaceInfo> info = boost::dynamic_pointer_cast<SpaceInfo>( _info.getSpecifics() );
  base_movement_center_ = movements::Pose( info->approximate_relative_object_position_, Eigen::Quaterniond(1,0,0,0) );
  /////////////////////////////// init stuff ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  return true;
}

View YoubotPlanner::getCurrentView()
{////////////////////////////////////////////////////////////////////////////////////////////////////////////
  return View();
}

bool YoubotPlanner::getPlanningSpace( ViewSpace* _space )
{////////////////////////////////////////////////////////////////////////////////////////////////////////////
  return false;
}

bool YoubotPlanner::getSubPlanningSpace( ViewSpace* _space, View& _view, double _distance )
{///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  return false;
}

RobotPlanningInterface::ReceiveInfo YoubotPlanner::retrieveData()
{
  makeScan(0.1);
  
  // check if remode issued data, if so return true ////////////////////////////////////////////////////////////////////////////////
  return RobotPlanningInterface::RECEIVED;
}

RobotPlanningInterface::MovementCost YoubotPlanner::calculateCost( View& _target_view )
{//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  return RobotPlanningInterface::MovementCost();
}

bool YoubotPlanner::moveTo( View& _target_view )
{//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  return false;
}

void YoubotPlanner::calculateArmGrid( double _y_res, double _z_res, std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& _joint_values, std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >* _grid )
{
  if( _y_res<=0 || _z_res<=0 )
  {
    throw std::invalid_argument("YoubotPlanner::calculateArmGrid:: Called with negative or zero resolution which is not permitted.");
  }
  
  /* from youbot specifications:
   * 
   * height of link 2 over ground: 0.140m (base height) + 0.147m (height of link2 relative to arm base) = 0.287m
   * max stretch from link 2 to link 5: 0.518m (arm base to link5) - 0.147m (link2 height) = 0.371m
   * max stretch to link5: 0.518m
   * max stretch to link4: 0.437m
   * max stretch from link 2 to link 4: 0.437 - 0.147 = 0.29m
   * 
   */
  double ground_safety_distance = 0.04; //[m]
  
  // distances relative to arm base
  double z_min = -0.14+ground_safety_distance;
  double z_max = 0.518;
  double y_min = -0.55; //actually x, since y is normal to the movement plane of links 2-4 in the system of link 2
  double y_max = 0.55;
  
  double z_step = 1/_z_res; // [m]
  double y_step = 1/_y_res; // [m]
  
  // constructing moveit robot state
  planning_scene_monitor::LockedPlanningSceneRO current_scene( scene_ );
  robot_state::RobotState robot_state = current_scene->getCurrentState();
  
  geometry_msgs::PoseStamped arm_base_transform = robot_->getCurrentPose("arm_link_1"); //T_b,a1
  geometry_msgs::PoseStamped arm_l5_transform = robot_->getCurrentPose("arm_link_5"); //T_b,a5
  Eigen::Matrix<double,3,4> T_ba5_1 = st_is::transformationMatrix( arm_l5_transform.pose ); //T_b,a5
  Eigen::Matrix<double,4,4> T_ba5;
  T_ba5<<T_ba5_1,0,0,0,1;
  Eigen::Matrix<double,3,4> T_ba1_1 = st_is::transformationMatrix( arm_base_transform.pose ); //T_b,a1
  Eigen::Matrix<double,4,4> T_ba1;
  T_ba1<<T_ba1_1,0,0,0,1;
  Eigen::Matrix<double,4,4> T_a1a5 = T_ba1.inverse()*T_ba5;
  ROS_INFO_STREAM("the y offset is: "<< T_a1a5(1,3) );
  
  Eigen::Vector3d arm_base_pos =  st_is::geometryToEigen( arm_base_transform.pose.position ); //b_trans_b,a1
  Eigen::Quaterniond arm_base_ori = st_is::geometryToEigen( arm_base_transform.pose.orientation ); // rot_b,a1
  
  int total_calculations=0;
  int success_count=0;
  
  ROS_INFO("Start calculation of achievable grid points.");
  // iteration...
  for( double z = z_min; z<=z_max; z+=z_step )
  {
    for( double y = y_min; y<=y_max; y+=y_step )
    {
      // reset arm to reference pose
      robot_state.setVariablePosition( "arm_joint_2", 1.15 );
      robot_state.setVariablePosition( "arm_joint_3", -2.6 );
      /*robot_state.setVariablePosition( "arm_joint_4", 3.4 );
      Eigen::Quaterniond link_4_reference_orientation( robot_state.getGlobalLinkTransform("arm_link_4").rotation() );
      
       // desired target pose
      Eigen::Vector3d rel_pos(y, 0, z); // in a1-system
      Eigen::Vector3d rel_pos_pf = arm_base_ori*rel_pos+arm_base_pos; // transform relative pose (which is relative to arm base) to planning frame
      
      geometry_msgs::Pose target_pose;
      target_pose.position = st_is::eigenToGeometry( rel_pos_pf );
      target_pose.orientation = st_is::eigenToGeometry( link_4_reference_orientation );
      
      
      // try to set target pose using inverse kinematics
      bool success = robot_state.setFromIK( robot_state.getJointModelGroup("arm"), target_pose, 1 );
      
      if( success )
      {
	ROS_WARN("IK succeeded.");
	ros::Duration(0.5).sleep();
	success_count++;
      }
      else
	ROS_INFO("IK failed.");
      
      
      continue;*/
      
      // desired target pose
      Eigen::Vector3d rel_pos(y, 0, z); // in a1-system
      Eigen::Vector3d rel_pos_pf = arm_base_ori*rel_pos; // transform relative pose (which is relative to arm base) to planning frame
      Eigen::Vector3d arm_link_5_pos = arm_base_pos + rel_pos_pf;
      
      // iterate through link 4 positions to find valuable reference pose for the given target, then keep orientation of link 5 fixed to find IK (current method needs orientation)
      int iteration_count = 0;
      double arm_4_pos_min = 0.03;
      
      for( double arm_4_pos = arm_4_pos_min; arm_4_pos<3.42; arm_4_pos+=0.1 )
      {
	total_calculations++;
	
	if( total_calculations%200==0 )
	  ROS_INFO("Still calculating...");
	  
	// prefer even and upright positions of end link...
	if( iteration_count<=3 )
	{
	  if( iteration_count==0 )
	    arm_4_pos = 3.4; // turned to the left
	  else if( iteration_count==1 )
	    arm_4_pos = 1.88; // upright
	  else if( iteration_count==2 )
	    arm_4_pos = 0.36; //turned to the right
	    
	  iteration_count++;
	}
	  
	robot_state.setVariablePosition( "arm_joint_4", arm_4_pos );
	Eigen::Quaterniond link_5_reference_orientation( robot_state.getGlobalLinkTransform("arm_link_5").rotation() );
	
	geometry_msgs::Pose target_pose;
	target_pose.position = st_is::eigenToGeometry( arm_link_5_pos );
	target_pose.orientation = st_is::eigenToGeometry( link_5_reference_orientation );
	
	// try to set target pose using inverse kinematics
	bool success = robot_state.setFromIK( robot_state.getJointModelGroup("arm"), target_pose, "arm_link_5", 1 );
	
	if( success )
	{
	  //ROS_WARN("IK succeeded.");
	  Eigen::Vector3d joint_positions;
	  joint_positions(0) = robot_state.getVariablePosition("arm_joint_2");
	  joint_positions(1) = robot_state.getVariablePosition("arm_joint_3");
	  joint_positions(2) = robot_state.getVariablePosition("arm_joint_4");
	  _joint_values.push_back(joint_positions);
	  
	  if( _grid!=nullptr ) // output the relative values as well
	  {
	    Eigen::Vector2d grid_coordinates;
	    grid_coordinates(0) = y;
	    grid_coordinates(1) = z;
	    _grid->push_back(grid_coordinates);
	  }
	  
	  success_count++;
	  break; // go to next point on success
	}
	//else
	//  ROS_INFO("IK failed.");
	
	if( iteration_count<=3 )
	  arm_4_pos = arm_4_pos_min; // start iteration
      }
            
    }
  }
  
  ROS_INFO_STREAM("Total calculations executed: "<<total_calculations);
  ROS_INFO_STREAM("Succeeded calculations: "<<success_count);
}

bool YoubotPlanner::calculateScanTrajectory( Eigen::Vector3d _joint_values, double _radius, std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& _joint_trajectory, int _planning_attempts )
{
  // constructing moveit robot state
  planning_scene_monitor::LockedPlanningSceneRO current_scene( scene_ );
  robot_state::RobotState robot_state = current_scene->getCurrentState();
  robot_state.setVariablePosition( "arm_joint_2", _joint_values(0) );
  robot_state.setVariablePosition( "arm_joint_3", _joint_values(1) );
  robot_state.setVariablePosition( "arm_joint_4", _joint_values(2) );
  
  //movements::Pose base_pose = linkPoseInRobotState("arm_link_5",robot_state);
  movements::Pose link_4_pose = linkPoseInRobotState("arm_link_4",robot_state);
  
  // adius, 4 turns/sec, 0.025 m/s radial speed ->2s to reach limit
  movements::KinMove scan = movements::InOutSpiral::create( link_4_pose.orientation, _radius, 4*6.283185307, 0.025, movements::InOutSpiral::ZXPlane );
  //start time, end time, time step size
  movements::PoseVector m_waypoints = scan.path( link_4_pose, 0.5, 3.5, 0.02 );
  
  double max_dropoff=0.2;
  
  bool successfully_planned = filteredPlanFromMovementsPathForRobotState( m_waypoints, "arm_link_4", robot_state, _joint_trajectory, 1000, max_dropoff );
  
  return successfully_planned;
}

movements::Pose YoubotPlanner::linkPoseInRobotState( std::string _name, robot_state::RobotState& _state )
{
  Eigen::Affine3d link_pose = _state.getGlobalLinkTransform(_name);
  Eigen::Quaterniond link_orientation( link_pose.rotation() );
  Eigen::Vector3d link_translation( link_pose.translation() );
  
  return movements::Pose( link_translation, link_orientation );
}

bool YoubotPlanner::filteredPlanFromMovementsPathForRobotState( const movements::PoseVector& _waypoints, std::string _link_name, const robot_state::RobotState& _state, std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& _joint_values, int _planning_attempts, double _max_dropoff )
{
  robot_state::RobotState state(_state);
  
  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > waypoints;
  std::vector< robot_state::RobotStatePtr> trajectory;
  
  //construct waypoints vector
  BOOST_FOREACH( auto pose, _waypoints )
  {
    geometry_msgs::Pose geo_pose = movements::toROS(pose);
    Eigen::Affine3d affine_pose;
    affine_pose.matrix() << st_is::transformationMatrix( geo_pose ),0,0,0,1;
    waypoints.push_back(affine_pose);
  }
  double success_ratio = 0;
  int count=0;
  int dropped_points = 0;
  int passed_points = _waypoints.size();
  
  
  while( success_ratio!=1 && ros_node_->ok() )
  {
    count++;
    success_ratio = state.computeCartesianPath( state.getJointModelGroup("arm"), trajectory, state.getLinkModel(_link_name), waypoints, true, 0.1, 0 );
    
    if( success_ratio!=1 && count>_planning_attempts ) // filter stage - only if necessary: a little complicated since working with vectors (which is given)
    {
      dropped_points++;
      
      if( dropped_points==passed_points || _max_dropoff<=0 )
      {
	return false;  // too many dropped points
      }
      else if( _max_dropoff<1 )
      {
	if( (double)dropped_points/passed_points > _max_dropoff )
	{
	  return false; // too many dropped points
	}
      }
      else
      {
	if( dropped_points > _max_dropoff )
	{
	  return false; // too many dropped points
	}
      }
      
      int valid_points = trajectory.size(); // -> also the index of the point which will be dropped
      count = 0;
      // drop point for which calculation failed
      for( unsigned int i=valid_points; i<waypoints.size()-1; i++ )
      {
	waypoints[i]=waypoints[i+1]; // shift all later points one position forward
      }
      waypoints.resize( waypoints.size()-1 ); // drop last
    }
    
  }
  // success!
  // fill output joint value path
  BOOST_FOREACH( auto robot_state, trajectory )
  {
    Eigen::Vector3d joint_values;
    joint_values(0) = robot_state->getVariablePosition("arm_joint_2");
    joint_values(1) = robot_state->getVariablePosition("arm_joint_3");
    joint_values(2) = robot_state->getVariablePosition("arm_joint_4");
    _joint_values.push_back(joint_values);
  }
  
  return true; 
}

bool YoubotPlanner::runSingleIteration()
{
  ros::spinOnce();
  
  return planAndMove();
}

movements::Pose YoubotPlanner::getEndEffectorPoseFromTF( ros::Duration _max_wait_time )
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

movements::Pose YoubotPlanner::getCurrentLinkPose( std::string _link )
{
  bool new_tf_available = tf_listener_.waitForTransform( base_planning_frame_,_link, ros::Time::now(), ros::Duration(3.0) );
  
  if( !new_tf_available )
    return movements::Pose();
  
  tf::StampedTransform end_effector_pose_tf;
  geometry_msgs::TransformStamped end_effector_pose_ros;
  
  try{
    tf_listener_.lookupTransform(base_planning_frame_, _link, ros::Time(0), end_effector_pose_tf);
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

bool YoubotPlanner::makeScan(double _max_dropoff)
{
  // only for the "fixed position scan":
  moveit::planning_interface::MoveGroup::Plan plan;
  if( spin_trajectory_ != nullptr )
  { // trajectory has already been computed or loaded before
    plan.trajectory_ = moveit_msgs::RobotTrajectory(*spin_trajectory_);
    remode_commander_.publish(remode_start_);
    executeMoveItPlan( plan );
    remode_commander_.publish(remode_stopandsmooth_);
    return true;
  }
  else
  {
    // try to load trajectory
    plan.trajectory_ = loadUpperArmTrajectory( "/home/stewss/Documents/YoubotTrajectories/YoubotSpinTrajectory_LM.traj" );
    if( !plan.trajectory_.joint_trajectory.points.empty() ) // got plan from file
    {
      ROS_INFO("Loaded trajectory from file.");
      spin_trajectory_ = boost::shared_ptr<moveit_msgs::RobotTrajectory>( new moveit_msgs::RobotTrajectory(plan.trajectory_) );
      remode_commander_.publish(remode_start_);
      executeMoveItPlan( plan );
      remode_commander_.publish(remode_stopandsmooth_);
      return true;
    }
  }
  ROS_INFO("Need to calculate trajectory...");
   // saveUpperArmTrajectoryPositions("/home/stewss/Documents/YoubotTrajectories/YoubotSpinTrajectory_LM.traj",trajectory);
  
  
  // for everything else:
  
  
  //geometry_msgs::Pose current_pose = robot_->getCurrentPose().pose;
  movements::Pose base_pose = getCurrentLinkPose("camera");//movements::fromROS(current_pose);
  
  //geometry_msgs::Pose link_4_pose = robot_->getCurrentPose("arm_link_4").pose;
  movements::Pose link_4 = getCurrentLinkPose("arm_link_4");//::fromROS(link_4_pose);
  // 0.05m radius, 4 turns/sec, 0.025 m/s radial speed ->2s to reach limit
  movements::KinMove scan = movements::InOutSpiral::create( link_4.orientation, 0.06, 4*6.283185307, 0.025, movements::InOutSpiral::ZXPlane );
  
  movements::PoseVector m_waypoints = scan.path( base_pose, 0.5, 3.5, 0.02 );
  
  // the camera should point into the same direction during the whole movement - seems not to have an impact
  moveit_msgs::OrientationConstraint eef_orientation_constraint = getFixedEEFLinkOrientationConstraint(base_pose);
  
  moveit_msgs::Constraints robot_constraints;
  robot_constraints.orientation_constraints.push_back(eef_orientation_constraint);
  
  remode_commander_.publish(remode_start_);
  bool scan_success = executeMovementsPath( m_waypoints, &robot_constraints, _max_dropoff );
  remode_commander_.publish(remode_stopandsmooth_);
  
  if( scan_success )
  {    
    geometry_msgs::Pose stop_pose = robot_->getCurrentPose().pose;
    movements::Pose stop_pose_m = movements::fromROS(stop_pose);
    movements::PoseVector go_home;
    go_home.push_back(stop_pose_m);
    go_home.push_back(base_pose);
    
    executeMovementsPath( go_home );
    
    return true;
  }
  else
    return false;
}

bool YoubotPlanner::moveBaseCircularlyTo( movements::Pose _target_position, movements::Pose _center, double _radial_speed )
{return false;
  movements::Pose base_pose = getCurrentLinkPose("base_footprint");
  movements::Pose object_center( Eigen::Vector3d(1,0,0), Eigen::Quaterniond() );
  //movements::Pose object_center( base_pose.position+2*base_pose.position.normalized(), Eigen::Quaterniond() );
  
  // start position, end position, angular speed, movement direction
  movements::KinMove circle_seg = movements::CircularGroundPath::create( base_pose.position, _target_position.position, _radial_speed, movements::CircularGroundPath::SHORTEST );
  
  double dt = 0.25;
  boost::shared_ptr<movements::CircularGroundPath> circles = boost::dynamic_pointer_cast< movements::CircularGroundPath >( circle_seg.operator->() );
  double time_for_move = circles->totalAngle( _center )/_radial_speed;
  movements::PoseVector waypoints = circle_seg.path( _center, 0, time_for_move, dt );
  
  bool base_successfully_moved = executeMovementsTrajectoryOnBase(waypoints,1);
  return false;
}

void YoubotPlanner::moveBaseTo( double _x, double _y, double _theta )
{
  ros::Publisher commander = ros_node_->advertise<geometry_msgs::Pose2D>("/base_controller/position_command",1);
  geometry_msgs::Pose2D command;
  command.x=_x;
  command.y=_y;
  command.theta=_theta;
  commander.publish(command);
}

bool YoubotPlanner::executeMovementsTrajectoryOnBase( movements::PoseVector& _path, double _dt )
{
  control_msgs::FollowJointTrajectoryGoal traj;
  
  traj.trajectory.joint_names.push_back("youbot_base/x");
  traj.trajectory.joint_names.push_back("youbot_base/y");
  traj.trajectory.joint_names.push_back("youbot_base/theta");
  
  double associated_time = 0;
  BOOST_FOREACH( auto pose, _path )
  {
    trajectory_msgs::JointTrajectoryPoint new_point;
    new_point.positions.push_back( pose.position.x() );
    new_point.positions.push_back( pose.position.y() );
    double angle;
    if( pose.orientation.z()>0 )
      angle = 2*acos( pose.orientation.w() );
    else
      angle = 2*acos( -pose.orientation.w() );
    new_point.positions.push_back( angle );
    new_point.time_from_start = ros::Duration(associated_time);
    
    traj.trajectory.points.push_back(new_point);
    associated_time+=_dt;
  }
  
  base_trajectory_sender_.sendGoal(traj);
  base_trajectory_sender_.waitForResult();
  
  if (base_trajectory_sender_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    return true;
  else
    return false;
}

bool YoubotPlanner::executeMovementsPath( movements::PoseVector& _path, moveit_msgs::Constraints* _constraints, double _max_dropoff )
{
  moveit::planning_interface::MoveGroup::Plan plan;
  bool successfully_planned = filteredPlanFromMovementsPath( _path, plan, _constraints, 10, _max_dropoff );
  
  if( successfully_planned )
  {
    return executeMoveItPlan(plan);
  }
  return false;
}

bool YoubotPlanner::executeMoveItPlan( moveit::planning_interface::MoveGroup::Plan& _plan )
{
  ros::AsyncSpinner spinner(1);  
  //scene_->unlockSceneRead();   
  spinner.start();
  robot_->execute(_plan);
  spinner.stop();
  //scene_->lockSceneRead();
  return true;
}

bool YoubotPlanner::isCollisionFree( planning_scene_monitor::LockedPlanningSceneRO& _scene, robot_state::RobotState& _robot )
{
  bool colliding = _scene->isStateColliding( _robot );
    
  return !colliding;
}

bool YoubotPlanner::planFromMovementsPath( movements::PoseVector& _waypoints, moveit::planning_interface::MoveGroup::Plan& _plan, moveit_msgs::Constraints* _path_constraints, int _planning_attempts )
{
        
    /*geometry_msgs::Pose current_pose = robot_->getCurrentPose().pose;
    using namespace std;
    cout<<endl<<"Current pose as given in planFromMovementsPath is:"<<endl<<current_pose<<endl<<endl;
    movements::Pose base_pose = movements::fromROS(current_pose);
    movements::RelativeMovement z_down = movements::Translation::create(0,0,-0.1);
    movements::KinMove md = movements::Linear::create(0,0,-1,1); // moving downwards with 1 m/s
            
    moveit_msgs::Constraints robot_constraints;
    robot_constraints.orientation_constraints.push_back(eef_orientation_constraint);*/
    
    if( _path_constraints!=nullptr )
    {
      robot_->setPathConstraints( *_path_constraints );
    }
    
    std::vector<geometry_msgs::Pose> waypoints = toROS(_waypoints);
    
    moveit_msgs::RobotTrajectory trajectory;
    //moveit::planning_interface::MoveGroup::Plan plan;
    
    double success_ratio = 0;
    int count=0;
    while( success_ratio!=1 && ros_node_->ok() )
    {
      count++;
      success_ratio=robot_->computeCartesianPath( waypoints, 0.1,0 /*0.2 = ~10 degree max dist in config space, 0 disables it*/, trajectory );
      ros::Duration(0.01).sleep();
      
      if( count>_planning_attempts ) return false;
    }
    
    /*using namespace std;
    cout<<endl<<"calculated path has size :"<<endl<<waypoints.size()<<".";
    cout<<endl<<"The poses in the path generated by movements are:";
    BOOST_FOREACH(auto pose, waypoints)
    {
      cout<<pose<<endl;
    }
    
    std::vector<trajectory_msgs::JointTrajectoryPoint> traj_joint_values = trajectory.joint_trajectory.points;
    std::vector<std::string> traj_joint_names = trajectory.joint_trajectory.joint_names;
    
    planning_scene_monitor::LockedPlanningSceneRO current_scene( scene_ );
    robot_state::RobotState robot_state = current_scene->getCurrentState();
    
    // print out transforms for the calculated trajectory path points
    cout<<endl<<"The moveit trajectory consists of "<<traj_joint_values.size()<<" points.";
    cout<<endl<<"The computed trajectory is:";
    BOOST_FOREACH( auto traj_joint_value, traj_joint_values )
    {
      robot_state.setVariablePositions( traj_joint_names, traj_joint_value.positions );
      Eigen::Matrix<double,4,4> state_transform = robot_state.getFrameTransform("camera").matrix();
      movements::Pose relative_pose_m( Eigen::Vector3d(state_transform.topRightCorner<3,1>()), Eigen::Quaterniond(state_transform.topLeftCorner<3,3>()) );
      geometry_msgs::Pose relative_pose = movements::toROS(relative_pose_m);
      cout<<endl<<relative_pose;
    }
    cout<<endl<<"Which is in joint space:";
    BOOST_FOREACH( auto traj_joint_value, traj_joint_values )
    {
      cout<<endl<<traj_joint_value;
    }
    cout<<endl<<endl;*/
    //cout<<endl<<"The computed trajectory is:"<<endl<<trajectory<<endl<<endl;
    
    // attempt to create velocities *************************************
    /*
    // First to create a RobotTrajectory object
    robot_state::RobotState current_robot_state = current_scene->getCurrentState();
    
    robot_trajectory::RobotTrajectory rt( current_robot_state.getRobotModel(), planning_group_);
    rt.setRobotTrajectoryMsg( current_robot_state, trajectory );
    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory);
    // ***********************************
    */
    //cout<<endl<<"The computed trajectory is:"<<endl<<trajectory<<endl<<endl;
    
    _plan.trajectory_ = trajectory;
    //_plan.planning_time_ = 0.1;
  
    robot_->clearPathConstraints();
    
    return true;
}

bool YoubotPlanner::filteredPlanFromMovementsPath( movements::PoseVector& _waypoints, moveit::planning_interface::MoveGroup::Plan& _plan, moveit_msgs::Constraints* _path_constraints, int _planning_attempts, double _max_dropoff )
{
    if( _path_constraints!=nullptr )
    {
      robot_->setPathConstraints( *_path_constraints );
    }
    
    std::vector<geometry_msgs::Pose> waypoints = toROS(_waypoints);
    
    moveit_msgs::RobotTrajectory trajectory;
    //moveit::planning_interface::MoveGroup::Plan plan;
   
    double success_ratio = 0;
    int count=0;
    int dropped_points = 0;
    int passed_points = _waypoints.size();
    
    while( success_ratio!=1 && ros_node_->ok() )
    {
      count++;
      success_ratio=robot_->computeCartesianPath( waypoints, 0.1,0 /*0.2 = ~10 degree max dist in config space, 0 disables it*/, trajectory );
      
      if( success_ratio!=1 && count>_planning_attempts ) // filter stage - only if necessary: a little complicated since working with vectors (which is given)
      {
	dropped_points++;
	
	if( dropped_points==passed_points || _max_dropoff<=0 )
	{
	  /*using namespace std;
	  cout<<endl<<"Total number of points in trajectory passed is: p="<<_waypoints.size();
	  cout<<endl<<"Percentage of successfully computed cartesian path s is: s="<<success_ratio;
	  cout<<endl<<"Number of computed points in cartesian trajectory is: c = "<<trajectory.joint_trajectory.points.size();
	  cout<<endl<<"s*p = "<<success_ratio*_waypoints.size();
	  cout<<endl<<"Number of dropped points: "<<dropped_points;
	  cout<<endl<<"Dropped point percentage: "<<dropped_points/(double)_waypoints.size();
	  cout<<endl<<endl;*/
	  return false;  // too many dropped points
	}
	else if( _max_dropoff<1 )
	{
	  if( (double)dropped_points/passed_points > _max_dropoff )
	  {
	    /*using namespace std;
	    cout<<endl<<"Total number of points in trajectory passed is: p="<<_waypoints.size();
	    cout<<endl<<"Percentage of successfully computed cartesian path s is: s="<<success_ratio;
	    cout<<endl<<"Number of computed points in cartesian trajectory is: c = "<<trajectory.joint_trajectory.points.size();
	    cout<<endl<<"s*p = "<<success_ratio*_waypoints.size();
	    cout<<endl<<"Number of dropped points: "<<dropped_points;
	    cout<<endl<<"Dropped point percentage: "<<dropped_points/(double)_waypoints.size();
	    cout<<endl<<endl;*/
	    return false; // too many dropped points
	  }
	}
	else
	{
	  if( dropped_points > _max_dropoff )
	  {
	    /*using namespace std;
	    cout<<endl<<"Total number of points in trajectory passed is: p="<<_waypoints.size();
	    cout<<endl<<"Percentage of successfully computed cartesian path s is: s="<<success_ratio;
	    cout<<endl<<"Number of computed points in cartesian trajectory is: c = "<<trajectory.joint_trajectory.points.size();
	    cout<<endl<<"s*p = "<<success_ratio*_waypoints.size();
	    cout<<endl<<"Number of dropped points: "<<dropped_points;
	    cout<<endl<<"Dropped point percentage: "<<dropped_points/(double)_waypoints.size();
	    cout<<endl<<endl;*/
	    return false; // too many dropped points
	  }
	}
	
	int valid_points = trajectory.joint_trajectory.points.size(); // -> also the index of the point which will be dropped
	count = 0;
	// drop point for which calculation failed
	for( unsigned int i=valid_points; i<waypoints.size()-1; i++ )
	{
	  waypoints[i]=waypoints[i+1]; // shift all later points one position forward
	}
	waypoints.resize( waypoints.size()-1 ); // drop last
      }
      
      //ros::Duration(0.001).sleep();
    }
      
    using namespace std;
    cout<<endl<<"Total number of points in trajectory passed is: p="<<_waypoints.size();
    cout<<endl<<"Percentage of successfully computed cartesian path s is: s="<<success_ratio;
    cout<<endl<<"Number of computed points in cartesian trajectory is: c = "<<trajectory.joint_trajectory.points.size();
    cout<<endl<<"s*p = "<<success_ratio*_waypoints.size();
    cout<<endl<<"Number of dropped points: "<<dropped_points;
    cout<<endl<<"Dropped point percentage: "<<dropped_points/(double)_waypoints.size();
    cout<<endl<<endl;
      
    _plan.trajectory_ = trajectory;
    //_plan.planning_time_ = 0.1;
    //saveUpperArmTrajectoryPositions("/home/stewss/Documents/YoubotTrajectories/YoubotSpinTrajectory_LM.traj",trajectory);
    robot_->clearPathConstraints();
    
    return true;
}

moveit_msgs::OrientationConstraint YoubotPlanner::getFixedEEFLinkOrientationConstraint( movements::Pose& _base_pose, int _weight, double _x_axis_tolerance, double _y_axis_tolerance, double _z_axis_tolerance )
{
  moveit_msgs::OrientationConstraint eef_orientation_constraint;
  eef_orientation_constraint.header.frame_id = "base_footprint";
  eef_orientation_constraint.link_name = end_effector_planning_frame_;
  eef_orientation_constraint.orientation = st_is::eigenToGeometry(_base_pose.orientation);
  eef_orientation_constraint.absolute_x_axis_tolerance = _x_axis_tolerance;
  eef_orientation_constraint.absolute_y_axis_tolerance = _y_axis_tolerance;
  eef_orientation_constraint.absolute_z_axis_tolerance = _z_axis_tolerance;
  eef_orientation_constraint.weight = _weight;
  
  return eef_orientation_constraint;
}


void YoubotPlanner::setEndEffectorPlanningFrame( std::string _name )
{
  if( robot_->setEndEffector(_name) )
  {
    end_effector_planning_frame_ = _name;
  }
}


moveit_msgs::RobotTrajectory YoubotPlanner::loadUpperArmTrajectory( std::string _filename )
{
  std::ifstream in(_filename, std::ifstream::in);
  
  double next_value;
  
  std::vector<std::string> link_names;
  link_names.push_back("arm_joint_2");
  link_names.push_back("arm_joint_3");
  link_names.push_back("arm_joint_4");
  link_names.push_back("arm_joint_5");
  
  trajectory_msgs::JointTrajectory traj;
  trajectory_msgs::JointTrajectoryPoint point;
  
  traj.joint_names = link_names;
  int i=0;
  bool is_time = true;
  
  while( in>>next_value )
  {
    ROS_INFO_STREAM("Adding joint position "<<i++<<": "<<next_value);
    
    if( point.positions.size()==0 && is_time )
    {
      point.time_from_start = ros::Duration(next_value);
      is_time = false;
    }
    else
      point.positions.push_back(next_value);
    
    if( point.positions.size()==4 )
    {
      traj.points.push_back(point);
      is_time = true;
      point.positions.clear();
    }
  }
  ROS_INFO_STREAM("Number of trajectory points extracted from file is: "<<traj.points.size() );
  if( point.positions.size()!=0 )
  {
    ROS_ERROR("YoubotPlanner::loadUpperArmTrajectory:: the trajectory file loaded did not contain the correct number of values, a point might be missing at the end. Continuing without it.");
  }
  
  moveit_msgs::RobotTrajectory new_trajectory;
  new_trajectory.joint_trajectory = traj;
  return new_trajectory;
}


void YoubotPlanner::saveUpperArmTrajectoryPositions( std::string _filename, const moveit_msgs::RobotTrajectory& _trajectory )
{
  std::ofstream out(_filename, std::ofstream::trunc);
  
  std::vector<std::string> link_names;
  link_names.push_back("arm_joint_2");
  link_names.push_back("arm_joint_3");
  link_names.push_back("arm_joint_4");
  link_names.push_back("arm_joint_5");
  
  std::vector<unsigned int> map;
  for( unsigned int i=0; i<link_names.size(); i++ )
  {
    for( unsigned int m=0; m<_trajectory.joint_trajectory.joint_names.size(); m++ )
    {
      if( link_names[i]==_trajectory.joint_trajectory.joint_names[m] )
      {
	map.push_back(m);
	break;
      }
    }
  }
  
  if( map.size()==4 )
  {
    BOOST_FOREACH( auto point, _trajectory.joint_trajectory.points )
    {
      out << point.time_from_start.toSec() <<" "<<point.positions[map[0]] <<" " << point.positions[map[1]] <<" " << point.positions[map[2]] <<" " << point.positions[map[3]] <<"\n";
    }
  }
  else
    ROS_ERROR("Tried to save trajectory data but the joint names provided appear to be wrong.");
  
  out.close();
}

void YoubotPlanner::initializeTF()
{
  // t_OW = t_OI*t_IW - O:dr_origin(=odom at initialization), W:world, I:image frame
  ros::Time now = ros::Time::now();
  while( !tf_listener_.waitForTransform("cam_pos","world",now, ros::Duration(1.0) )&&ros_node_->ok() )
  {
    ROS_INFO("Waiting for 'cam_pos' to be published...");
    ros::spinOnce();
    now = ros::Time::now();
  }
  tf::StampedTransform t_WI;
  tf_listener_.lookupTransform("world","cam_pos",now,t_WI);
  
  geometry_msgs::Transform arm2image;
  bool hand_eye_available;
  do{
    hand_eye_available = ros_node_->getParam("/hec/arm2image/translation/x", arm2image.translation.x )
              && ros_node_->getParam("/hec/arm2image/translation/y", arm2image.translation.y )
              && ros_node_->getParam("/hec/arm2image/translation/z", arm2image.translation.z )
              && ros_node_->getParam("/hec/arm2image/rotation/x", arm2image.rotation.x )
              && ros_node_->getParam("/hec/arm2image/rotation/y", arm2image.rotation.y )
              && ros_node_->getParam("/hec/arm2image/rotation/z", arm2image.rotation.z )
              && ros_node_->getParam("/hec/arm2image/rotation/w", arm2image.rotation.w );
    if(!hand_eye_available)
    {
      ROS_INFO("Waiting for '/hec/arm2image' params to be available on parameter server...");
      ros::Duration(1).sleep();
      ros::spinOnce();
    }
  }while(!hand_eye_available&&ros_node_->ok());
  tf::Transform t_IA;
  tf::transformMsgToTF(arm2image,t_IA);
  
  // t_OA - A:last arm link
  now = ros::Time::now();
  while( !tf_listener_.waitForTransform("odom","arm_link_5",now, ros::Duration(1.0) ) &&ros_node_->ok())
  {
    ROS_INFO("Waiting for 'odom' to 'arm_link_5' being published...");
    now = ros::Time::now();
    ros::spinOnce();
  }
  tf::StampedTransform t_AO;
  tf_listener_.lookupTransform("arm_link_5","odom",now,t_AO);
  
  tf::Transform t_WO = t_WI*t_IA*t_AO;
  
  geometry_msgs::Transform wo_msg;
  tf::transformTFToMsg( t_WO, wo_msg );
  
  dense_reconstruction::PoseSetter request;
  request.request.pose = wo_msg;
  
  while( !ros::service::call("dense_reconstruction/set_world_pose",request)&&ros_node_->ok() )
  {
    ROS_INFO("Sending request to setup tf links to tf_linker, no answer...");
    ros::Duration(1).sleep();
    ros::spinOnce();
  }
  return;
}

bool YoubotPlanner::planAndMove()
{
  /*std::string end_effector_name;
  end_effector_name = robot_->getEndEffector();
  std::cout<<std::endl<<"end effector name: "<<end_effector_name;
  std::cout<<std::endl<<"planning frame name: "<<robot_->getPlanningFrame();
  std::cout<<std::endl<<"base position: "<<std::endl<<robot_->getCurrentPose("base_footprint")<<std::endl;
  movements::Pose basePose = getCurrentLinkPose("base_footprint");
  std::cout<<std::endl<<"base position translation from tf: "<<std::endl<<basePose.position<<std::endl;*/
    
  using namespace std;
  
  robot_->setGoalPositionTolerance(0.001);
  robot_->setGoalOrientationTolerance(0.001);

  cout<<endl<<"Press q to quit, s to scan again, m to move to the next position."<<endl;
  char input;
  cin>>input;
  if( input=='q' )
    return false;
  else if( input=='s' )
  {
    while( !makeScan(0.1) ){ros::spinOnce();}
  }
  else if( input=='m' )
  {
    movements::Pose base_pose = getCurrentLinkPose("base_footprint");
    movements::Pose object_center( Eigen::Vector3d(1,0,0), Eigen::Quaterniond() );
    //movements::Pose object_center( base_pose.position+2*base_pose.position.normalized(), Eigen::Quaterniond() );
    
    // start position, end position, angular speed, movement direction
    movements::KinMove circle_seg = movements::CircularGroundPath::create( base_pose.position, base_pose.position, base_radial_speed_, movements::CircularGroundPath::COUNTER_CLOCKWISE );
    
    double dt = 0.25;
    double move_time = base_move_angle_/base_radial_speed_;
    movements::PoseVector waypoints = circle_seg.path( base_movement_center_, 0, 1.25, dt );
    
    bool base_successfully_moved = executeMovementsTrajectoryOnBase(waypoints,1);
    //while( !makeScan(0.1) ){ros::spinOnce();};
  }
  
  return true;
}

std::string YoubotPlanner::SpaceInfo::type()
{
  return "YoubotPlanningSpaceInfo";
}

}