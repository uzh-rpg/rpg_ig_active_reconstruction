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
#include <angles/angles.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <boost/foreach.hpp>
#include "dense_reconstruction/flying_stereo_camera_interface.h"
#include "utils/ros_eigen.h"
#include <geometry_msgs/Pose2D.h>
#include "dense_reconstruction/PoseSetter.h"
#include "dense_reconstruction/remode_data_retriever.h"
#include "dense_reconstruction/stereo_camera_data_retriever.h"
#include "gazebo_msgs/SetModelState.h"

namespace dense_reconstruction
{

FlyingStereoCameraInterface::FlyingStereoCameraInterface( ros::NodeHandle* _n )
  :ros_node_(_n)
  ,tf_listener_(*_n)
{
  std::string interface_namespace="/flying_stereo_camera_interface/";
  
  data_retreiver_ = boost::shared_ptr<YoubotPlanner::DataRetrievalModule>( new StereoCameraDataRetriever() );
  
  
  view_planning_frame_="dr_origin";
  ros::param::get(interface_namespace+"view_planning_frame", view_planning_frame_);
  
  std::string data_folder, view_space_name;
  ros::param::get(interface_namespace+"view_planning_frame", data_folder);
  ros::param::get(interface_namespace+"view_planning_frame", view_space_name);
  
  view_space_.loadFromFile( data_folder+"/"+view_space_name);
  if( view_space_.size()==0 )
  {
    ROS_FATAL("The view space couldn't be loaded from file. Shutting down node...");
    ros::shutdown();
  }
  
  planning_space_initialization_server_ = ros_node_->advertiseService("/dense_reconstruction/robot_interface/planning_space_initialization", &FlyingStereoCameraInterface::planningSpaceInitService, this );
  feasible_view_space_request_server_ = ros_node_->advertiseService("/dense_reconstruction/robot_interface/feasible_view_space", &FlyingStereoCameraInterface::feasibleViewSpaceRequestService, this );
  current_view_server_ = ros_node_->advertiseService("/dense_reconstruction/robot_interface/current_view", &FlyingStereoCameraInterface::currentViewService, this );
  retrieve_data_server_ = ros_node_->advertiseService("/dense_reconstruction/robot_interface/retrieve_data", &FlyingStereoCameraInterface::retrieveDataService, this );
  movement_cost_server_ = ros_node_->advertiseService("/dense_reconstruction/robot_interface/movement_cost", &FlyingStereoCameraInterface::movementCostService, this );
  move_to_server_ = ros_node_->advertiseService("/dense_reconstruction/robot_interface/move_to", &FlyingStereoCameraInterface::moveToService, this );
  setup_tf_server_ = ros_node_->advertiseService("/dense_reconstruction/robot_interface/setup_tf", &FlyingStereoCameraInterface::setupTFService, this );
    
}

FlyingStereoCameraInterface::~FlyingStereoCameraInterface()
{
}

std::string FlyingStereoCameraInterface::initializePlanningFrame()
{
  return view_planning_frame_;
}

bool FlyingStereoCameraInterface::initializePlanningSpace( PlanningSpaceInitializationInfo& _info )
{
  ROS_WARN("FlyingStereoCameraInterface::initializePlanningSpace:: For this interface no planning space initialization is currently available. The planning space is loaded from file on startup.");
  
  return false;
}

View FlyingStereoCameraInterface::getCurrentView()
{  
  return view_space_.getView(current_view_);
}

bool FlyingStereoCameraInterface::getPlanningSpace( ViewSpace* _space )
{
  _space = &view_space_;
  
  return true;
}

RobotPlanningInterface::ReceiveInfo FlyingStereoCameraInterface::retrieveData()
{
  return data_retreiver_->retrieveData();
}

RobotPlanningInterface::MovementCost FlyingStereoCameraInterface::movementCost( View& _target_view )
{
  RobotPlanningInterface::MovementCost cost;
  cost.cost = 0;  
  return cost;
}

RobotPlanningInterface::MovementCost FlyingStereoCameraInterface::movementCost( View& _start_view, View& _target_view, bool _fill_additional_information )
{
  RobotPlanningInterface::MovementCost cost;
  cost.cost = 0;  
  return cost;
}

bool FlyingStereoCameraInterface::moveTo( View& _target_view )
{
  gazebo_msgs::SetModelState srv_call;
  srv_call.request.model_state.model_name = "flying_stereo_cam";
  srv_call.request.model_state.pose = movements::toROS( _target_view.pose() );
  
  ros::service::call( "/gazebo/set_model_state", srv_call );
  
  current_view_ = _target_view.index;
  
  ros::Duration(1.0).sleep();
  
  return true;
}

bool FlyingStereoCameraInterface::planningSpaceInitService( dense_reconstruction::PlanningSpaceInitializationInfoMsg::Request& _req, dense_reconstruction::PlanningSpaceInitializationInfoMsg::Response& _res )
{
  
  PlanningSpaceInitializationInfo general_info;  
  _res.accepted = initializePlanningSpace(general_info);
  
  return true;
}

bool FlyingStereoCameraInterface::feasibleViewSpaceRequestService( dense_reconstruction::FeasibleViewSpaceRequest::Request& _req, dense_reconstruction::FeasibleViewSpaceRequest::Response& _res )
{
  ROS_INFO("View space service called.");
  _res.view_space = view_space_.toMsg();
  return true;
}

bool FlyingStereoCameraInterface::currentViewService( dense_reconstruction::ViewRequest::Request& _req, dense_reconstruction::ViewRequest::Response& _res )
{
  _res.view = getCurrentView().toMsg();
    
  return true;
}

bool FlyingStereoCameraInterface::retrieveDataService( dense_reconstruction::RetrieveData::Request& _req, dense_reconstruction::RetrieveData::Response& _res )
{
  ROS_INFO("Data retrieval service called.");
  _res.receive_info = data_retreiver_->retrieveData();
  return true;
}

bool FlyingStereoCameraInterface::movementCostService( dense_reconstruction::MovementCostCalculation::Request& _req, dense_reconstruction::MovementCostCalculation::Response& _res )
{
  ROS_INFO("Movement cost service called.");
  View start(_req.start_view );
  View target(_req.target_view);
  
  MovementCost cost = movementCost( start, target, _req.additional_information );
  _res.movement_cost = cost.toMsg();
  return true;
}

bool FlyingStereoCameraInterface::moveToService( dense_reconstruction::MoveToOrder::Request& _req, dense_reconstruction::MoveToOrder::Response& _res )
{
  ROS_INFO("MoveTo service called.");
  View target( _req.target_view );
  _res.success = moveTo(target);
  return true;
}

bool FlyingStereoCameraInterface::setupTFService( std_srvs::Empty::Request& _req, std_srvs::Empty::Response& _res )
{
}

}