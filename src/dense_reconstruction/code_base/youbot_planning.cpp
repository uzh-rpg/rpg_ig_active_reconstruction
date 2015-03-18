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
#include <sstream>
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

YoubotPlanner::YoubotPlanner( ros::NodeHandle* _n )
  :ros_node_(_n)
  ,tf_listener_(*_n)
  ,base_trajectory_sender_("/base_controller/follow_joint_trajectory", true)
  ,plan_base_in_global_frame_(true)
{
  data_folder_set_ = ros_node_->getParam("/youbot_interface/data_folder",data_folder_);
  if( !data_folder_set_ )
    ROS_WARN("No data folder was set on parameter server. Precomputed arm configurations will not be loaded or stored.");
  
  planning_group_ = "arm";
  std::string remode_control_topic = "/remode/command";
  
  //moveit::planning_interface::MoveGroup::Options mg_options(planning_group_,"robot_description",*ros_node_);
  robot_ = boost::shared_ptr<moveit::planning_interface::MoveGroup>( new moveit::planning_interface::MoveGroup(planning_group_) );
  
  //setEndEffectorPlanningFrame("camera");
  plan_base_in_global_frame_ = false;
  base_planning_frame_="odom";
  view_planning_frame_="dr_origin";
  
  scene_ = boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>( new planning_scene_monitor::PlanningSceneMonitor("robot_description") );
  scene_->startStateMonitor();
  scene_->startSceneMonitor();
  scene_->startWorldGeometryMonitor();
  
  
  // block everything until complete tf tree is available: this doesn't help
  //tf_listener_.waitForTransform( "/base_footprint","/camera", ros::Time::now(), ros::Duration(0.0) );
  
  remode_commander_ = ros_node_->advertise<std_msgs::String>( remode_control_topic, 1 );
  remode_start_.data="SET_NEXT_FRAME_IS_REFERENCE";
  remode_stopandsmooth_.data="SMOOTH_AND_STOP_UPDATING";
  remode_has_published_=false;
  max_remode_wait_time_ = ros::Duration(3.0); // 3 seconds
  remode_topic_subsriber_ = ros_node_->subscribe( "/remode/pointcloud",1, &dense_reconstruction::YoubotPlanner::remodeCallback, this );
  
  // base movement initialization
  base_move_angle_ = 0.1*6.283185307; // [rad] one tenth of a circle
  base_radial_speed_ = 0.1*6.283185307; // [rad/s]
  base_movement_center_ = movements::Pose( Eigen::Vector3d(1,0,0), Eigen::Quaterniond() );
  
  
  // move arm into initial pose
  assumeAndSetInitialPose();
}

YoubotPlanner::~YoubotPlanner()
{
  if( init_view_!=nullptr )
    delete init_view_;
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
  
  return view_planning_frame_;
}

bool YoubotPlanner::initializePlanningSpace( PlanningSpaceInitializationInfo& _info )
{
  if( _info.getSpecifics()->type()!="YoubotPlanningSpaceInfo" )
  {
    ROS_ERROR_STREAM("YoubotPlanner::initializePlanningSpace::The PlanningSpaceInitializationInfo is of type '"<<_info.getSpecifics()->type()<<"'. It should be of type 'YoubotPlanningSpaceInfo'. Initialization is not possible.");
    return false;
  }
  ROS_INFO("Initializing ViewSpace for the given parameters...");
  
  boost::shared_ptr<SpaceInfo> info = boost::dynamic_pointer_cast<SpaceInfo>( _info.getSpecifics() );
  if( base_movement_center_.position == Eigen::Vector3d(0,0,0) )
  {
    ROS_ERROR("YoubotPlanner::initializePlanningSpace::The provided base_movement_center_ (0,0,0) says that the object resides exactly at the position of the robot which is not possible.");
    return false;
  }
  base_movement_center_ = movements::Pose( info->approximate_relative_object_position_, Eigen::Quaterniond(1,0,0,0) );
  scan_radius_ = info->scan_radius_;
  
  // build planning space
  std::vector< BaseConfig,Eigen::aligned_allocator<BaseConfig> > base_space;
  std::vector<Link1Config> link1_space;
  std::vector<ArmConfig> arm_space;
  std::vector<Link5Config> link5_space;
  
  // initialize spaces
  getBaseSpace( info, base_space );
  getLink1Space( info, link1_space );
  getArmSpace( info, arm_space );
  getLink5Space( info, link5_space );
  
  if( base_space.size()==0 || link1_space.size()==0 || arm_space.size()==0 || link5_space.size()==0 )
  {
    ROS_ERROR("Initializing ViewSpace for the given parameters failed. One or more joint spaces are empty. Please check your parameter setting or contact your technical advisor.");
    return false;
  }
  
  ROS_INFO("Constructing space..");
  // initialize viewpoint space (view_point_data_) with all possible combinations
  BOOST_FOREACH( auto base_config, base_space )
  {
    BOOST_FOREACH( auto link1_config, link1_space )
    {
      BOOST_FOREACH( auto arm_config, arm_space )
      {
	BOOST_FOREACH( auto link5_config, link5_space )
	{
	  ViewPointData new_vp_data;
	  new_vp_data.base_config_ = base_config;
	  new_vp_data.link1_config_ = link1_config;
	  new_vp_data.arm_config_ = arm_config;
	  new_vp_data.link5_config_ = link5_config;
	  
	  new_vp_data.pose_ = calculateCameraPoseFromConfiguration(new_vp_data);
	  
	  view_point_data_.push_back(new_vp_data);
	}
      }
    }
  }
  
  ROS_INFO_STREAM("View space setup and constructed with "<<view_point_data_.size()<<" view points.");
  
  return true;
}

View YoubotPlanner::getCurrentView()
{ 
  if( current_view_!=nullptr )
    return viewFromViewPointData( *current_view_ );
  
  // else: get view from current pose...
  View current_view(view_planning_frame_);
  current_view.pose() = getCurrentGlobalLinkPose("cam_pos");
  
  return current_view;
}

bool YoubotPlanner::getPlanningSpace( ViewSpace* _space )
{
  if( _space==nullptr )
    return false;
  
  for( unsigned int i=0;i<view_point_data_.size();i++ )
  {
    (*_space).push_back( viewFromViewPointData(view_point_data_[i]) );
  }
  
  return true;
}

/*bool YoubotPlanner::getSubPlanningSpace( ViewSpace* _space, View& _view, double _distance )
{
  return false;
}*/

RobotPlanningInterface::ReceiveInfo YoubotPlanner::retrieveData()
{
  if( current_view_==nullptr )
  {
    // TODO in unknown pose: replan!
    ROS_WARN("YoubotPlanner::retrieveData::Cannot retrieve data because current view is unknown and thus no scanning movement is available. Retrieving data failed.");
    return RobotPlanningInterface::RECEPTION_FAILED;
  }
  
  moveit::planning_interface::MoveGroup::Plan scan;
  getFullMotionPlan( *current_view_, scan );
    
  if( scan.trajectory_.joint_trajectory.points.size()!=0 )
  {
    remode_commander_.publish(remode_start_);
    bool remode_has_published_ = false;
    
    executeMoveItPlan( scan );
    
    remode_commander_.publish(remode_stopandsmooth_);
    
    ros::Time publish_time = ros::Time::now();
    while( !remode_has_published_ && ros_node_->ok() ) // wait for remode to publish
    {
      if( ros::Time::now() > (publish_time+max_remode_wait_time_) )
      {
	ROS_WARN("No data retrieved in time.");
	break; // no data received, waited long enough
      }
      ros::Duration(0.005).sleep();
      ros::spinOnce();
    }
    
    if( remode_has_published_ )
    {
      return RobotPlanningInterface::RECEIVED;
    }
    else
    {
      return RobotPlanningInterface::RECEPTION_FAILED;
    }
  }
  else
  {
    ROS_WARN("YoubotPlanner::retrieveData::Cannot retrieve data because the scanning view path associated with the current view was empty. Retrieving data failed.");
    return RobotPlanningInterface::RECEPTION_FAILED;
  }
  
}

RobotPlanningInterface::MovementCost YoubotPlanner::calculateCost( View& _target_view )
{
  if( current_view_==nullptr )
  {
    RobotPlanningInterface::MovementCost cost;
    cost.exception = RobotPlanningInterface::MovementCost::INVALID_STATE;
    return cost; // TODO: just read current configuration
  }
  
  // using distances between base poses as current measure
  movements::Pose current = (*current_view_).base_config_.pose_;
  
  ViewPointData* referenced_view_point = getCorrespondingData(_target_view);
  movements::Pose target = (*referenced_view_point).base_config_.pose_;
  
  Eigen::Vector3d direct_path = current.position - target.position;
  
  RobotPlanningInterface::MovementCost cost;
  cost.cost = direct_path.norm();
  return cost;
}

bool YoubotPlanner::moveTo( View& _target_view )
{
  ViewPointData* referenced_view_point = getCorrespondingData(_target_view);
  
  bool successfully_moved;
  
  // move arm
  std::vector<double> arm_config(5);
  arm_config[0] = (*referenced_view_point).link1_config_.angle_;
  arm_config[1] = (*referenced_view_point).arm_config_.j2_angle_;
  arm_config[2] = (*referenced_view_point).arm_config_.j3_angle_;
  arm_config[3] = (*referenced_view_point).arm_config_.j4_angle_;
  arm_config[4] = (*referenced_view_point).link5_config_.angle_;
  successfully_moved = assumeArmPosition(arm_config);
    
  // move base
  movements::Pose base_target_pos = (*referenced_view_point).base_config_.pose_;
  
      std::cout<<std::endl<<"base pose to move to is:";
      std::cout<<std::endl<<"position:";
      std::cout<<std::endl<<"x:"<<base_target_pos.position.x();
      std::cout<<std::endl<<"y:"<<base_target_pos.position.y();
      std::cout<<std::endl<<"z:"<<base_target_pos.position.z();
  movements::Pose base_current_pos_rpf = getCurrentLinkPose("base_footprint"); // robot (moveit) planning frame
  movements::Pose base_current_pos;
  try
  {
    if( plan_base_in_global_frame_ )
      base_current_pos = moveitPlanningFrameToViewPlanningFrame( base_current_pos_rpf );
    else
      base_current_pos = base_current_pos_rpf;
  }
  catch(std::runtime_error& e)
  {
    ROS_WARN_STREAM("YoubotPlanner::moveTo::"<<e.what()<<".. Using local moveit planning frame instead.");
    base_current_pos = base_current_pos_rpf;
  }
  successfully_moved = successfully_moved && moveBaseCircularlyTo( base_target_pos, base_movement_center_, base_radial_speed_ );
  
  if( successfully_moved )
    current_view_ = referenced_view_point;
  else current_view_ = nullptr; // movement unsuccessful, pose possibly unknown
  
  return successfully_moved;
}

bool YoubotPlanner::assumeAndSetInitialPose()
{
  ROS_INFO("Assuming initial pose...");
  
  double j1_pose = 1.35;
  double j2_pose = 0.7;
  double j3_pose = -1.0;
  double j4_pose = 2.2;
  double j5_pose = 2.9;
  
  ros_node_->getParam("/youbot_interface/init/joint_1",j1_pose);
  ros_node_->getParam("/youbot_interface/init/joint_2",j2_pose);
  ros_node_->getParam("/youbot_interface/init/joint_3",j3_pose);
  ros_node_->getParam("/youbot_interface/init/joint_4",j4_pose);
  ros_node_->getParam("/youbot_interface/init/joint_5",j5_pose);
  
  std::vector<double> initial_pose(5);
  initial_pose[0] = j1_pose;
  initial_pose[1] = j2_pose;
  initial_pose[2] = j3_pose;
  initial_pose[3] = j4_pose;
  initial_pose[4] = j5_pose;
  bool pose_assumed = assumeArmPosition(initial_pose);
  
  movements::Pose base_pose_rpf = getCurrentLinkPose("base_footprint");
  movements::Pose base_pose_vpf;
  try
  {
    base_pose_vpf = moveitPlanningFrameToViewPlanningFrame(base_pose_rpf);
  }
  catch(std::runtime_error& e)
  {
    ROS_WARN_STREAM("YoubotPlanner::assumeAndSetInitialPose: "<<e.what()<<". Using coordinates from odometry directly instead.");
    base_pose_vpf = base_pose_rpf;
  }
  
  Eigen::Vector3d arm_pos(j2_pose,j3_pose,j4_pose);
  boost::shared_ptr< std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > > trajectory( new std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >() );
  
  if( !calculateScanTrajectory(arm_pos, scan_radius_, *trajectory, 5) )
  {
    ROS_WARN("Could not compute scan trajectory for initial pose, data retrieval might not be possible without moving to another pose first.");
  }
  
  init_view_ = new ViewPointData();
  init_view_->base_config_.pose_ = base_pose_vpf;
  init_view_->link1_config_.angle_ = j1_pose;
  init_view_->arm_config_.j2_angle_ = j2_pose;
  init_view_->arm_config_.j3_angle_ = j3_pose;
  init_view_->arm_config_.j4_angle_ = j4_pose;
  init_view_->arm_config_.scan_trajectory_ = trajectory;
  init_view_->link5_config_.angle_ = j5_pose;
  init_view_->pose_ = calculateCameraPoseFromConfiguration(*init_view_);
  
  current_view_ = init_view_;
  
  return pose_assumed;
}

bool YoubotPlanner::assumeArmPosition( const std::vector<double>& _joint_values )
{
  if( _joint_values.size()!=5 )
    return false;
  
  robot_->setJointValueTarget("arm_joint_1", _joint_values[0] );
  robot_->setJointValueTarget("arm_joint_2", _joint_values[1] );
  robot_->setJointValueTarget("arm_joint_3", _joint_values[2] );
  robot_->setJointValueTarget("arm_joint_4", _joint_values[3] );
  robot_->setJointValueTarget("arm_joint_5", _joint_values[4] );
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // plan and execute a path to the target state
  bool successfully_moved = robot_->move();
  spinner.stop();
  
  return successfully_moved;
}

YoubotPlanner::ViewPointData* YoubotPlanner::getCorrespondingData( View& _view )
{
  // find view point data corresponding to target view  
  boost::shared_ptr<View::ViewInfo> info_reference = _view.associatedData();
  
  if( info_reference->type()!="YoubotViewPointInfo" )
  {
    std::string error = "YoubotPlanner::getCorrespondingData::Called with view that didn't belong to Youbot. Associated data type was '"+info_reference->type()+"' instead of 'YoubotViewPointInfo'.";
    throw std::runtime_error(error);
  }
  boost::shared_ptr<YoubotPlanner::ViewInfo> info = boost::dynamic_pointer_cast<YoubotPlanner::ViewInfo>(info_reference);
  
  return info->getViewPointData();
}

void YoubotPlanner::getFullMotionPlan( ViewPointData& _view, moveit::planning_interface::MoveGroup::Plan& _plan )
{
  // _plan.trajectory_ = moveit_msgs::RobotTrajectory.joint_trajectory.
  _plan.trajectory_.joint_trajectory.joint_names.resize(5);
  _plan.trajectory_.joint_trajectory.joint_names[0]="arm_joint_1";
  _plan.trajectory_.joint_trajectory.joint_names[1]="arm_joint_2";
  _plan.trajectory_.joint_trajectory.joint_names[2]="arm_joint_3";
  _plan.trajectory_.joint_trajectory.joint_names[3]="arm_joint_4";
  _plan.trajectory_.joint_trajectory.joint_names[4]="arm_joint_5";
  
  double j1_pos = _view.link1_config_.angle_;
  double j5_pos = _view.link5_config_.angle_;
  double associated_time = 0;
  double dt = 0.2;
  
  BOOST_FOREACH( auto arm_config, *_view.arm_config_.scan_trajectory_ )
  {
    trajectory_msgs::JointTrajectoryPoint new_point;
    new_point.positions.resize(5);
    new_point.positions[0] = j1_pos;
    new_point.positions[1] = arm_config(0);
    new_point.positions[2] = arm_config(1);
    new_point.positions[3] = arm_config(2);
    new_point.positions[4] = j5_pos;
    new_point.time_from_start = ros::Duration(associated_time);
    _plan.trajectory_.joint_trajectory.points.push_back(new_point);
    associated_time+=dt; // TODO: the computeCartesianPath function of the RobotState class seems to have no notion of time, unlike the movegroup class - investigate if better values could be found!
  }
  
  return;
}

movements::Pose YoubotPlanner::calculateCameraPoseFromConfiguration( ViewPointData& _config )
{
  // geometry_msgs::Transform arm2image_
  // calculate pose of link5 in world, given the configuration
  // hand eye transformation
  Eigen::Matrix<double,3,4> t_IE = st_is::transformationMatrix(arm2image_);
  Eigen::Matrix<double,4,4> t_IE_h;
  t_IE_h << t_IE, 0, 0, 0, 1;
  Eigen::Matrix<double,4,4> t_EI = t_IE_h.inverse();
  
  // base pose
  geometry_msgs::Pose p_GB = movements::toROS(_config.base_config_.pose_);
  Eigen::Matrix<double,3,4> t_GB_uh = st_is::transformationMatrix(p_GB);
  Eigen::Matrix<double,4,4> t_GB;
  t_GB << t_GB_uh, 0, 0, 0, 1;
  
  // constructing moveit robot state
  planning_scene_monitor::LockedPlanningSceneRO current_scene( scene_ );
  robot_state::RobotState robot_state = current_scene->getCurrentState();
  robot_state.setVariablePosition( "arm_joint_1", _config.link1_config_.angle_ );
  robot_state.setVariablePosition( "arm_joint_2", _config.arm_config_.j2_angle_ );
  robot_state.setVariablePosition( "arm_joint_3", _config.arm_config_.j3_angle_ );
  robot_state.setVariablePosition( "arm_joint_4", _config.arm_config_.j4_angle_ );
  robot_state.setVariablePosition( "arm_joint_5", _config.link5_config_.angle_ );
  // calculate transform from arm_link_5 to base_footprint for this robot state
  
  Eigen::Affine3d t_OB = robot_state.getGlobalLinkTransform("base_footprint");
  Eigen::Affine3d t_BO = t_OB.inverse();
  Eigen::Affine3d t_OE = robot_state.getGlobalLinkTransform("arm_link_5");
  
  Eigen::Affine3d t_BE = t_BO*t_OE;
  
  // pose of image frame I relative to view planning frame G
  Eigen::Affine3d t_GI = Eigen::Affine3d(t_GB)*t_BE*Eigen::Affine3d(t_EI);
  
  Eigen::Vector3d G_trans_GI( t_GI.translation() );
  Eigen::Quaterniond r_GI( t_GI.rotation() );
  
  return movements::Pose(G_trans_GI,r_GI);
}

View YoubotPlanner::viewFromViewPointData( ViewPointData& _vpd )
{
  View new_view;
  new_view.pose() = _vpd.pose_;
  new_view.sourceFrame() = view_planning_frame_;
  boost::shared_ptr<ViewInfo> reference( new ViewInfo(_vpd) );
  new_view.associatedData() = reference;
  return new_view;
}

void YoubotPlanner::getArmSpaceDescriptions( double _radius, double _min_view_distance, double _view_resolution, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& _joint_space, std::vector<boost::shared_ptr<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > > >& _joint_trajectories )
{
  
  // filter out grid points for which no scanning is possible, get trajectories for those points for which scanning is possible
  std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > possible_joint_values;
  std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > possible_coordinates;
  std::vector<boost::shared_ptr<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > > > trajectories;
  
  bool recalculate_arm_space = true;
  if(data_folder_set_) // try to load data from file
  {
    if( loadArmSpaceDescriptionsFromFile(_radius,_view_resolution,possible_joint_values,possible_coordinates,trajectories) )
    {
      ROS_INFO_STREAM("Complete arm space informations for radius = "<<_radius<<" and view resolution = "<<_view_resolution<<" found on disk: Successfully loaded.");
      recalculate_arm_space = false;
    }
  }
  if(recalculate_arm_space)
  {
    ROS_INFO_STREAM("No arm space information for the given  configuration radius = "<<_radius<<" and view resolution = "<<_view_resolution<<" found on disk. New configuration is being calculated.");
    // probably not the most effective way for calculating but fast to program
    
    // get arm grid
    std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > joint_value_grid;
    std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > coordinate_grid;
    getArmGrid( _view_resolution, joint_value_grid, coordinate_grid );
      
      
    int nr_of_planning_attempts = 1; // the robot state cartesian path computation method looks deterministic
    ROS_INFO_STREAM("Calculating trajectories for arm grid which consists of "<<joint_value_grid.size()<<" points. Currently at most "<<nr_of_planning_attempts<<" attempts are performed for each point. Points with unfeasible trajectories are disregarded.");
    ros::Duration(5).sleep();
    for( unsigned int i=0; i<joint_value_grid.size(); i++ )
    {
      auto joint_config = joint_value_grid[i];
      auto trajectory = boost::shared_ptr< std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >( new std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >() );
      ROS_INFO_STREAM("Calculating trajectory for grid point "<<i<<"/"<<joint_value_grid.size()<<"... Currently "<<possible_joint_values.size()<<" feasible configurations have been found.");
      if( calculateScanTrajectory(joint_config,_radius,*trajectory,nr_of_planning_attempts) )
      {
	possible_joint_values.push_back(joint_config);
	trajectories.push_back(trajectory);
	possible_coordinates.push_back(coordinate_grid[i]);
	ROS_INFO_STREAM("Success! This is a feasible configuration for scanning movements, configuration is added.");
	ros::Duration(1).sleep();
      }
      else
      {
	ROS_INFO_STREAM("Trajectory could not be computed for this configuration, configuration is dropped.");
	ros::Duration(1).sleep();
      }
    }
    
    if( possible_joint_values.size()!=0 )
    {
      ROS_INFO("Grid calculation succeeded.");
      if( data_folder_set_ )
      {
	ROS_INFO("Attempting to save calculated data.");
	if( saveArmSpaceDescriptionsToFile(_radius,_view_resolution,possible_joint_values, possible_coordinates, trajectories) )
	{
	  ROS_INFO("Data saved.");
	}
	else
	  ROS_INFO("Failed to save data.");
      }
    }
    else
      ROS_INFO("Grid calculation failed.");
    
  }
  // filter data by distance
  std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > valid_joint_values;
  std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > valid_coordinates;
  std::vector<boost::shared_ptr<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > > > final_trajectories;
  
  ROS_INFO_STREAM(""<<possible_joint_values.size()<<" feasible arm grid points were found. Filtering by min distance...");
  ros::Duration(5).sleep();
  
  for( unsigned int i=0; i<possible_joint_values.size(); i++ )
  {
    bool is_not_too_close = true;
    for( unsigned int a=0; a<valid_joint_values.size(); a++ )
    {
      Eigen::Vector2d dist = possible_coordinates[i]-valid_coordinates[a];
      is_not_too_close = ( dist.norm()>=_min_view_distance );
      if(!is_not_too_close)
	break; // already invalid
    }
    if( is_not_too_close )
    {
      valid_joint_values.push_back(possible_joint_values[i]);
      valid_coordinates.push_back(possible_coordinates[i]);
      final_trajectories.push_back(trajectories[i]);
    }
  }
  
  _joint_space = valid_joint_values;
  _joint_trajectories = final_trajectories;
  
  ROS_INFO_STREAM( valid_joint_values.size()<<" points are left in final arm grid." );
  
}

bool YoubotPlanner::loadArmSpaceDescriptionsFromFile( double _radius, double _view_resolution, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& _joint_space, std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& _coordinates, std::vector<boost::shared_ptr<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > > >& _joint_trajectories )
{
  std::string file_name;
  std::stringstream file_name_str;
  
  file_name_str << data_folder_<<"YoubotArmPosPreCalc_"<<_view_resolution<<"_"<<"InOutSpiral"<<"_"<<_radius<<".youbotarm";
  file_name=file_name_str.str();
  
  std::ifstream in(file_name, std::ifstream::in);
  
  int nr_of_points;
  if( !(in>>nr_of_points) )
    return false;
  
  for( unsigned int i=0; i<nr_of_points ; i++ )
  {
    Eigen::Vector3d joint_angles;
    
    bool successfully_read=true;
    
    successfully_read = successfully_read && ( in>>joint_angles(0) );
    successfully_read = successfully_read && ( in>>joint_angles(1) );
    successfully_read = successfully_read && ( in>>joint_angles(2) );
    
    
    _joint_space.push_back(joint_angles);
    
    Eigen::Vector2d grid_coordinates;
    
    successfully_read = successfully_read && ( in>>grid_coordinates(0) );
    successfully_read = successfully_read && ( in>>grid_coordinates(1) );
    
    _coordinates.push_back(grid_coordinates);
    
    int nr_of_traj_points;
    successfully_read = successfully_read && ( in>>nr_of_traj_points );
    
    auto trajectory = boost::shared_ptr<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >( new std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >() );
    
    for( unsigned int t=0; t<nr_of_traj_points; t++ )
    {
      Eigen::Vector3d traj_point;
    
      successfully_read = successfully_read && ( in>>traj_point(0) );
      successfully_read = successfully_read && ( in>>traj_point(1) );
      successfully_read = successfully_read && ( in>>traj_point(2) );
      
      (*trajectory).push_back( traj_point );
    }
    
    if( !successfully_read )
    {
      _joint_space.clear();
      _coordinates.clear();
      _joint_trajectories.clear();
      return false;
    }
    _joint_trajectories.push_back( trajectory );
    
  }
  
  in.close();
  
  return true;
}

bool YoubotPlanner::saveArmSpaceDescriptionsToFile( double _radius, double _view_resolution, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& _joint_space, std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& _coordinates, std::vector<boost::shared_ptr<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > > >& _joint_trajectories )
{
  std::string file_name;
  std::stringstream file_name_str;
  
  file_name_str << data_folder_<<"YoubotArmPosPreCalc_"<<_view_resolution<<"_"<<"InOutSpiral"<<"_"<<_radius<<".youbotarm";
  file_name=file_name_str.str();
  
  std::ofstream out(file_name, std::ofstream::trunc);
  
  
  int nr_of_points = _joint_space.size();
  if( !(out<<nr_of_points) )
    return false;
  if( _joint_trajectories.size()!=nr_of_points || nr_of_points!=_coordinates.size() )
    return false;
  
  for( unsigned int i=0; i<nr_of_points ; i++ )
  {
    Eigen::Vector3d joint_angles = _joint_space[i];
    
    bool successfully_wrote=true;
    
    successfully_wrote = successfully_wrote && ( out<<" "<<joint_angles(0) );
    successfully_wrote = successfully_wrote && ( out<<" "<<joint_angles(1) );
    successfully_wrote = successfully_wrote && ( out<<" "<<joint_angles(2) );
    
    Eigen::Vector2d coordinates = _coordinates[i];
    successfully_wrote = successfully_wrote && ( out<<" "<<coordinates(0) );
    successfully_wrote = successfully_wrote && ( out<<" "<<coordinates(1) );
    
    int nr_of_traj_points = (*_joint_trajectories[i]).size();
    successfully_wrote = successfully_wrote && ( out<<" "<<nr_of_traj_points );
        
    auto trajectory = _joint_trajectories[i];
    
    for( unsigned int t=0; t<nr_of_traj_points; t++ )
    {
      Eigen::Vector3d traj_point = (*trajectory)[t];
    
      successfully_wrote = successfully_wrote && ( out<<" "<<traj_point(0) );
      successfully_wrote = successfully_wrote && ( out<<" "<<traj_point(1) );
      successfully_wrote = successfully_wrote && ( out<<" "<<traj_point(2) );
    }
    
    if( !successfully_wrote )
    {
      return false;
    }
    
  }
  
  out.close();
  
  return true;
}

void YoubotPlanner::getArmGrid( double _resolution, std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& _joint_values, std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& _coordinates )
{
  if(data_folder_set_)
  {
    if( loadArmGridFromFile(_resolution,_joint_values,_coordinates) )
    {
      ROS_INFO_STREAM("Found calculated youbot arm grid with resolution "<<_resolution<<" on file and loaded it successfully.");
      return;
    }
  }
  ROS_INFO_STREAM("Found no fitting grid on file, calculating new one.");
  calculateArmGrid( _resolution,_resolution,_joint_values,&_coordinates );
  
  if( _joint_values.size()!=0 )
  {
    ROS_INFO_STREAM("Successfully calculated arm grid with "<<_joint_values.size()<<" points.");
    if(data_folder_set_)
    {
      ROS_INFO_STREAM("Saving it to disk.");
      if( saveArmGridToFile( _resolution,_joint_values,_coordinates ) )
	ROS_INFO("Grid saved.");
      else
	ROS_INFO("Saving the grid failed.");
    }
  }
  else
    ROS_INFO_STREAM("Arm grid calculation failed, no fitting grid points were found.");
  ros::Duration(5).sleep();
}

bool YoubotPlanner::loadArmGridFromFile( double _resolution, std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& _joint_values, std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& _coordinates )
{
  // arm joints
  std::string file_name;
  std::stringstream file_name_str;
  
  file_name_str << data_folder_<<"YoubotArmGrid_Res"<<_resolution<<".youbotarm";
  file_name=file_name_str.str();
  
  std::ifstream in(file_name, std::ifstream::in);
  
  bool read_successfully = true;
  
  int nr_of_points;
  read_successfully = read_successfully && (in>>nr_of_points);
  
  for( unsigned int i=0; i<nr_of_points; i++ )
  {
    Eigen::Vector3d joint_value_point;
    
    read_successfully = read_successfully && (in>>joint_value_point(0));
    read_successfully = read_successfully && (in>>joint_value_point(1));
    read_successfully = read_successfully && (in>>joint_value_point(2));
    
    if(!read_successfully)
    {
      _joint_values.clear();
      return false;
    }
    _joint_values.push_back(joint_value_point);
  }
  
  in.close();
  
  // arm coordinates
  std::string file_name2;
  std::stringstream file_name2_str;
  
  file_name2_str << data_folder_<<"YoubotArmGridCoordinates_Res"<<_resolution<<".youbotarm";
  file_name2=file_name2_str.str();
  
  std::ifstream in2(file_name2, std::ifstream::in);
  
  int nr_of_coordinates;
  
  read_successfully = read_successfully && (in2>>nr_of_coordinates);
  
  if( !read_successfully || nr_of_coordinates!=_joint_values.size() )
  {
    _joint_values.clear();
    return false;
  }
  
  for( unsigned int i=0; i<nr_of_coordinates; i++ )
  {
    Eigen::Vector2d coord;
    read_successfully = read_successfully && (in2>>coord(0));
    read_successfully = read_successfully && (in2>>coord(1));
    
    if(!read_successfully)
    {
      _joint_values.clear();
      _coordinates.clear();
      return false;
    }
    
    _coordinates.push_back(coord);
  }
  
  in2.close();
    
  return true;
}

bool YoubotPlanner::saveArmGridToFile( double _resolution, std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& _joint_values, std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& _coordinates )
{
  if( _coordinates.size()!=_joint_values.size() )
    return false;
  
  std::string file_name;
  std::stringstream file_name_str;
  
  file_name_str << data_folder_<<"YoubotArmGrid_Res"<<_resolution<<".youbotarm";
  file_name=file_name_str.str();
  
  std::ofstream out(file_name, std::ofstream::trunc);
  
  if( !(out<<_joint_values.size()<<"\n") )
    return false;
  
  BOOST_FOREACH( auto value, _joint_values )
  {
    if( !(out<<value(0)<<" "<<value(1)<<" "<<value(2)<<"\n") )
    {
      return false;
    }
  }
  out.close();
  
  
  std::string file_name2;
  std::stringstream file_name2_str;
  
  file_name2_str << data_folder_<<"YoubotArmGridCoordinates_Res"<<_resolution<<".youbotarm";
  file_name2=file_name2_str.str();
  
  std::ofstream out2(file_name2, std::ofstream::trunc);
  
  if( !(out2<<_coordinates.size()<<"\n") )
    return false;
  
  BOOST_FOREACH( auto value, _coordinates )
  {
    if( !(out2<<value(0)<<" "<<value(1)<<" "<<"\n") )
    {
      return false;
    }
  }
  out2.close();
  
  return true;
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
	else if( total_calculations==10118 )
	  ROS_INFO("Yes, still calculating. Why is that so surprising?");
	  
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
  
  movements::Pose eef_pose = linkPoseInRobotState("camera",robot_state);
  movements::Pose link_4_pose = linkPoseInRobotState("arm_link_4",robot_state);
  
  // adius, 4 turns/sec, 0.025 m/s radial speed ->2s to reach limit
  movements::KinMove scan = movements::InOutSpiral::create( link_4_pose.orientation, _radius, 4*6.283185307, 0.025, movements::InOutSpiral::ZXPlane );
  //start time, end time, time step size // calculate for end effector pose since that is the link for which cartesian path are planned by moveit
  movements::PoseVector m_waypoints = scan.path( eef_pose, 0.5, 3.5, 0.02 );
  
  double max_dropoff=0.2;
  bool successfully_planned;
  int count=0;
  do
  {
    count++;
    if( count%5==0)
      ROS_INFO_STREAM("Attempt "<<count<<"/"<<_planning_attempts);
    successfully_planned = filteredPlanFromMovementsPathForRobotState( m_waypoints, "camera", robot_state, _joint_trajectory, 1, max_dropoff, false );
  }while(!successfully_planned && count<_planning_attempts );
  
  
  return successfully_planned;
}

movements::Pose YoubotPlanner::linkPoseInRobotState( std::string _name, robot_state::RobotState& _state )
{
  Eigen::Affine3d link_pose = _state.getGlobalLinkTransform(_name);
  Eigen::Quaterniond link_orientation( link_pose.rotation() );
  Eigen::Vector3d link_translation( link_pose.translation() );
  
  return movements::Pose( link_translation, link_orientation );
}

bool YoubotPlanner::filteredPlanFromMovementsPathForRobotState( const movements::PoseVector& _waypoints, std::string _link_name, const robot_state::RobotState& _state, std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& _joint_values, int _planning_attempts, double _max_dropoff, bool _verbose )
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
  int total_it_count = 0;
  
  //robot_->setStartState(_state);
  //std::vector<geometry_msgs::Pose> waypoints = movements::toROS(_waypoints);
  //moveit_msgs::RobotTrajectory trajectory;
  
  while( success_ratio!=1 && ros_node_->ok() )
  {
    total_it_count++;
    count++;
    //success_ratio=robot_->computeCartesianPath( waypoints, 0.1,0 /*0.2 = ~10 degree max dist in config space, 0 disables it*/, trajectory );
    success_ratio = state.computeCartesianPath( state.getJointModelGroup("arm"), trajectory, state.getLinkModel(_link_name), waypoints, true, 0.1, 0 );
    if( total_it_count%10==0 && _verbose )
    {
      ROS_INFO_STREAM("Attempting to compute cartesian path with "<<waypoints.size()<<" waypoints. Current success ratio is "<<success_ratio<<", "<<dropped_points<<" points have been dropped out of "<<passed_points<<" which accounts for a dropoff of "<<100.0*dropped_points/(double)passed_points<<"%.");
    }
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
  //robot_->setStartStateToCurrentState();
  return true; 
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

movements::Pose YoubotPlanner::getCurrentGlobalLinkPose( std::string _link )
{
  bool new_tf_available = tf_listener_.waitForTransform( view_planning_frame_,_link, ros::Time::now(), ros::Duration(3.0) );
  
  if( !new_tf_available )
    return movements::Pose();
  
  tf::StampedTransform end_effector_pose_tf;
  geometry_msgs::TransformStamped end_effector_pose_ros;
  
  try{
    tf_listener_.lookupTransform(view_planning_frame_, _link, ros::Time(0), end_effector_pose_tf);
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

movements::Pose YoubotPlanner::moveitPlanningFrameToViewPlanningFrame( movements::Pose& _moveit_pose )
{
  ros::Time now = ros::Time::now();
  bool new_tf_available = tf_listener_.waitForTransform( view_planning_frame_,base_planning_frame_, now, ros::Duration(3.0) );
  
  if( !new_tf_available )
  {
    std::string message = "YoubotPlanner::moveitPlanningFrameToViewPlanningFrame::Transform from child "+base_planning_frame_+" to source "+view_planning_frame_+" couldn't be retrieved in time.";
    throw std::runtime_error(message);
  }
  tf::StampedTransform base_pose_tf;
  geometry_msgs::TransformStamped base_pose_ros;
  
  try{
    tf_listener_.lookupTransform(view_planning_frame_, base_planning_frame_, now, base_pose_tf);
  }
  catch (tf::TransformException ex){
    std::string message = std::string("YoubotPlanner::moveitPlanningFrameToViewPlanningFrame::Tf error occured when retrieving the transform: ")+std::string(ex.what());
    throw std::runtime_error(message);
  }
  tf::transformStampedTFToMsg( base_pose_tf, base_pose_ros );
  
  Eigen::Quaterniond rotation = st_is::geometryToEigen( base_pose_ros.transform.rotation );
  Eigen::Vector3d translation = st_is::geometryToEigen( base_pose_ros.transform.translation );
  
  Eigen::Quaterniond out_rotation = rotation*_moveit_pose.orientation;
  Eigen::Vector3d out_position = rotation*_moveit_pose.position + translation;
  
  return movements::Pose( out_position, out_rotation );
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
{
  movements::Pose base_pose_rpf = getCurrentLinkPose("base_footprint"); // robot (moveit) planning frame
  movements::Pose base_pose = moveitPlanningFrameToViewPlanningFrame(base_pose_rpf);
    
  // start position, end position, angular speed, movement direction
  movements::KinMove circle_seg = movements::CircularGroundPath::create( base_pose.position, _target_position.position, _radial_speed, movements::CircularGroundPath::SHORTEST );
  
  double dt = 0.25;
  boost::shared_ptr<movements::CircularGroundPath> circles = boost::dynamic_pointer_cast< movements::CircularGroundPath >( circle_seg.operator->() );
  double time_for_move = circles->totalAngle( _center )/_radial_speed;
  movements::PoseVector waypoints = circle_seg.path( _center, 0, time_for_move, dt );
  
  bool base_successfully_moved = executeMovementsTrajectoryOnBase(waypoints,1);
  
  return base_successfully_moved;
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

void YoubotPlanner::remodeCallback( const sensor_msgs::PointCloud2ConstPtr& _msg )
{
  remode_has_published_ = true;
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
  arm2image_ = arm2image;
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

void YoubotPlanner::getBaseSpace( boost::shared_ptr<SpaceInfo> _info, std::vector< BaseConfig,Eigen::aligned_allocator<BaseConfig> >& _config )
{
  ROS_INFO("Setting up joint space for base...");
  
  double min_radius = _info->base_min_radius_;
  double max_radius = _info->base_max_radius_;
  unsigned int base_circle_traj_nr = _info->base_circle_traj_nr_;
  unsigned int base_pts_per_circle = _info->base_pts_per_circle_;
  
  std::vector<double> radius_set;
  
  if( max_radius<min_radius || base_circle_traj_nr<=1 )
  {
    radius_set.push_back(min_radius);
  }
  else
  {
    double step_size = (max_radius-min_radius)/(base_circle_traj_nr-1);
    for( double radius = min_radius; radius<=max_radius; radius+=step_size )
    {
      radius_set.push_back(radius);
    }
  }
  
  if( base_pts_per_circle<=0 )
    base_pts_per_circle=8;
  
  movements::Pose current_base_pos_rpf = getCurrentLinkPose("base_footprint");
  movements::Pose current_base_pos;
  try
  {
    if( plan_base_in_global_frame_ )
      current_base_pos = moveitPlanningFrameToViewPlanningFrame( current_base_pos_rpf ); // ok, not really necessary...
    else
      current_base_pos = current_base_pos_rpf;
  }
  catch(std::runtime_error& e)
  {
    ROS_WARN_STREAM("Problem occured while setting up space for robot base: "<<e.what()<<". Using odometry coordinates as fallback.");
    current_base_pos = current_base_pos_rpf;
  }
  movements::Pose movement_center = base_movement_center_;
  Eigen::Vector3d c2b = current_base_pos.position - movement_center.position; // vector center to base
  c2b.normalize();
    
  
  BOOST_FOREACH( auto radius, radius_set )
  {
    movements::Pose circle_start = movement_center;
    circle_start.position = movement_center.position + radius*c2b;
    
    // construct circle movement with 2pi per sec radial speed
    movements::KinMove circle = movements::CircularGroundPath::create( circle_start.position, circle_start.position, 2*3.141592654, movements::CircularGroundPath::COUNTER_CLOCKWISE );
    
    double time_step = 1.0/base_pts_per_circle;
    for( double time = 0; time<1; time+=time_step )
    {
      BaseConfig new_config;
      new_config.pose_ = movement_center + circle(time);
      _config.push_back(new_config);
      std::cout<<std::endl<<"base pose is:";
      std::cout<<std::endl<<"position:";
      std::cout<<std::endl<<"x:"<<new_config.pose_.position.x();
      std::cout<<std::endl<<"y:"<<new_config.pose_.position.y();
      std::cout<<std::endl<<"z:"<<new_config.pose_.position.z();
    }
  }
}

void YoubotPlanner::getLink1Space( boost::shared_ptr<SpaceInfo> _info, std::vector<Link1Config>& _config )
{
  ROS_INFO("Setting up joint space for arm link 1.");
  
  double link_1_min_angle = _info->link_1_min_angle_;
  double link_1_max_angle = _info->link_1_max_angle_;
  unsigned int link_1_nr_of_pos = _info->link_1_nr_of_pos_;
  
  std::vector<double> angle_set;
  
  if( link_1_nr_of_pos<=1 || link_1_max_angle<link_1_min_angle )
  {
    angle_set.push_back(link_1_min_angle);
  }
  else
  {
    double step_size = (link_1_max_angle-link_1_min_angle)/(link_1_nr_of_pos-1);
    for( double angle = link_1_min_angle; angle<=link_1_max_angle; angle+=step_size )
    {
      angle_set.push_back(angle);
    }
  }
  
  BOOST_FOREACH( auto angle, angle_set )
  {
    Link1Config new_config;
    new_config.angle_ = angle;
    _config.push_back(new_config);
  }
}

void YoubotPlanner::getArmSpace( boost::shared_ptr<SpaceInfo> _info, std::vector<ArmConfig>& _config )
{
  ROS_INFO("Setting up joint space for arm links 2, 3 and 4.");
  
  double scan_radius = _info->scan_radius_;
  double arm_min_view_distance = _info->arm_min_view_distance_;
  
  double arm_view_resolution = _info->arm_view_resolution_;
  
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > joint_space;
  std::vector<boost::shared_ptr<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > > > scan_trajectories;
  
  getArmSpaceDescriptions( scan_radius, arm_min_view_distance, arm_view_resolution, joint_space, scan_trajectories );
  
  for( unsigned int i=0; i<joint_space.size(); i++ )
  {
    ArmConfig new_config;
    new_config.j2_angle_ = joint_space[i](0);
    new_config.j3_angle_ = joint_space[i](1);
    new_config.j4_angle_ = joint_space[i](2);
    new_config.scan_trajectory_ = scan_trajectories[i];
    _config.push_back(new_config);
  }
  
}

void YoubotPlanner::getLink5Space( boost::shared_ptr<SpaceInfo> _info, std::vector<Link5Config>& _config )
{
  ROS_INFO("Setting up joint space for arm link 5.");
  
  double link_5_min_angle = _info->link_5_min_angle_;
  double link_5_max_angle = _info->link_5_max_angle_;
  unsigned int link_5_nr_of_pos = _info->link_5_nr_of_pos_;
  
  std::vector<double> angle_set;
  
  if( link_5_nr_of_pos<=1 || link_5_max_angle<link_5_min_angle )
  {
    angle_set.push_back(link_5_min_angle);
  }
  else
  {
    double step_size = (link_5_nr_of_pos-link_5_min_angle)/(link_5_nr_of_pos-5);
    for( double angle = link_5_min_angle; angle<=link_5_max_angle; angle+=step_size )
    {
      angle_set.push_back(angle);
    }
  }
  
  BOOST_FOREACH( auto angle, angle_set )
  {
    Link5Config new_config;
    new_config.angle_ = angle;
    _config.push_back(new_config);
  }
}

YoubotPlanner::SpaceInfo::SpaceInfo()
  :base_min_radius_(1)
  ,base_max_radius_(2)
  ,base_circle_traj_nr_(1)
  ,base_pts_per_circle_(8)
  ,link_1_min_angle_(1.35)
  ,link_1_max_angle_(1.45)
  ,link_1_nr_of_pos_(1)
  ,scan_radius_(0.05)
  ,arm_min_view_distance_(0.2)
  ,arm_view_resolution_(10)
  ,link_5_min_angle_(2.9)
  ,link_5_max_angle_(3.4)
  ,link_5_nr_of_pos_(1)
{
  approximate_relative_object_position_<<0,0,0;
}

std::string YoubotPlanner::SpaceInfo::type()
{
  return "YoubotPlanningSpaceInfo";
}

YoubotPlanner::ViewInfo::ViewInfo()
{
  
}

YoubotPlanner::ViewInfo::ViewInfo( ViewPointData& _reference )
{
  view_point_ = &_reference;
}

std::string YoubotPlanner::ViewInfo::type()
{
  return "YoubotViewPointInfo";
}

YoubotPlanner::ViewPointData* YoubotPlanner::ViewInfo::getViewPointData()
{
  return view_point_;
}

}