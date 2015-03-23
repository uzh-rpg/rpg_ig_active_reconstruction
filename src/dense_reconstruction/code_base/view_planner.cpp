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

#include "dense_reconstruction/view_planner.h"


namespace dense_reconstruction
{

ViewPlanner::ViewPlanner( ros::NodeHandle& _n )
  :nh_(_n)
  ,start_(false)
  ,pause_(false)
  ,stop_and_print_(false)
  ,reinit_(false)
{
  
  view_space_retriever_ = nh_.serviceClient<dense_reconstruction::FeasibleViewSpaceRequest>("/dense_reconstruction/robot_interface/feasible_view_space");
  current_view_retriever_ = nh_.serviceClient<dense_reconstruction::ViewRequest>("/dense_reconstruction/robot_interface/current_view");
  data_retriever_ = nh_.serviceClient<dense_reconstruction::RetrieveData>("/dense_reconstruction/robot_interface/retrieve_data");
  cost_retriever_ = nh_.serviceClient<dense_reconstruction::MovementCostCalculation>("/dense_reconstruction/robot_interface/movement_cost");
  view_information_retriever_ = nh_.serviceClient<dense_reconstruction::ViewInformationReturn>("/dense_reconstruction/3d_model/information");
  robot_mover_ = nh_.serviceClient<dense_reconstruction::MoveToOrder>("/dense_reconstruction/robot_interface/move_to");
    
  planning_frame_ = "dr_origin";
  metrics_to_use_.push_back("NrOfUnknownVoxels");
  metrics_to_use_.push_back("AverageUncertainty");
  metrics_to_use_.push_back("AverageEndPointUncertainty");
  metrics_to_use_.push_back("UnknownObjectSideFrontier");
  metrics_to_use_.push_back("UnknownObjectVolumeFrontier");
  metrics_to_use_.push_back("ClassicFrontier");
  metrics_to_use_.push_back("EndNodeOccupancySum");
  metrics_to_use_.push_back("TotalOccupancyCertainty");
  metrics_to_use_.push_back("TotalNrOfOccupieds");
  
  command_ = nh_.subscribe( "/dense_reconstruction/view_planner/command", 1, &ViewPlanner::commandCallback, this );
}

void ViewPlanner::run()
{
  while( !start_ ) // wait for start signal
  {
    waitAndSpin();
  }
  // get view space from robot_interface
  while( !getViewSpace() )
  {
    ROS_INFO("View space service not available yet. Waiting...");
    waitAndSpin(2);
  }
  
  // get current view
  while( !getCurrentView(current_view_) )
  {
    ROS_INFO("Attempting to retrieve start view. Waiting...");
    waitAndSpin(2);
  }
  
  // gather initial data
  RobotPlanningInterface::ReceiveInfo receive_info;
  bool receive_service_succeeded = false;
  do
  {
    receive_service_succeeded = retrieveData(receive_info);
  }while( !receive_service_succeeded || receive_info!=RobotPlanningInterface::RECEIVED );
  
  // enter loop
  do
  {
    // possibly build subspace of complete space
    std::vector<unsigned int> views_to_consider;
    determineAvailableViewSpace( views_to_consider );
    
    // get movement costs
    std::vector<double> cost(views_to_consider.size());
    for( unsigned int i=0; i<cost.size(); i++ )
    {
      RobotPlanningInterface::MovementCost cost_description;
      View target = view_space_.getView( views_to_consider[i] );
      movementCost( cost_description, current_view_, target );
      
      if( cost_description.exception!=RobotPlanningInterface::MovementCost::NONE )
      {
	view_space_.setBad( views_to_consider[i] ); // don't consider that view
      }
      else
      {
	cost[i] = cost_description.cost;
      }
    }
    
    // get expected informations for each
    std::vector< std::vector<double> > information(views_to_consider.size());
    for( unsigned int i=0; i<information.size(); i++ )
    {
      View target_view = view_space_.getView( views_to_consider[i] );
      
      movements::PoseVector target_positions;
      target_positions.push_back( target_view.pose() );
      
      getViewInformation( information[i], target_positions );
    }
    
  }while(!stop_and_print_);
  
  /////////////////// PRINT INFORMATION ////////////////////////
}

void ViewPlanner::waitAndSpin(double _sec)
{
  ros::Duration(0.5).sleep();
  ros::spinOnce();
}

void ViewPlanner::determineAvailableViewSpace( std::vector<unsigned int>& _output )
{
  view_space_.getGoodViewSpace(_output);
}

bool ViewPlanner::getViewSpace()
{
  FeasibleViewSpaceRequest request;
  
  bool response = view_space_retriever_.call(request);
  
  if( response )
  {
    view_space_.fromMsg( request.response.view_space );
  }
  
  return response;
}

bool ViewPlanner::getCurrentView( View& _output)
{
  ViewRequest request;
  
  bool response = current_view_retriever_.call(request);
  
  if( response )
  {
    View out( request.response.view );
    _output = out;
  }
  
  return response;
}

bool ViewPlanner::retrieveData( RobotPlanningInterface::ReceiveInfo& _output )
{
  RetrieveData request;
  
  bool response = data_retriever_.call(request);
  
  if( response )
  {
    _output = RobotPlanningInterface::ReceiveInfo( request.response.receive_info );
  }
  
  return response;
}

bool ViewPlanner::movementCost( RobotPlanningInterface::MovementCost& _output, View& _start_view, View& _target_view )
{
  MovementCostCalculation request;
  request.request.start_view = _start_view.toMsg();
  request.request.target_view = _target_view.toMsg();
  
  bool response = cost_retriever_.call(request);
  
  if( response )
  {
    _output.fromMsg( request.response.movement_cost );
  }
  
  return response;
}

bool ViewPlanner::moveTo( bool& _output, View& _target_view )
{
  MoveToOrder request;
  request.request.target_view = _target_view.toMsg();
  
  bool response = data_retriever_.call(request);
  
  if( response )
  {
    _output = request.response.success;
  }
  
  return response;
}

bool ViewPlanner::getViewInformation( std::vector<double>& _output, movements::PoseVector& _poses )
{
  ViewInformationReturn request;
  request.request.call.poses = movements::toROS(_poses);
  request.request.call.metric_names = metrics_to_use_;
  
  request.request.call.ray_resolution_x = 0.5;
  request.request.call.ray_resolution_y = 0.5;
  request.request.call.ray_step_size = 2;
  
  double subwindow_width = 188; // [px]
  double subwindow_height = 120; // [px]
  request.request.call.max_x = 376 + subwindow_width/2;
  request.request.call.min_x = 376 - subwindow_width/2;
  request.request.call.max_y = 240 + subwindow_height/2;
  request.request.call.min_y = 240 - subwindow_height/2;
  
  request.request.call.min_ray_depth = 0.05;
  request.request.call.max_ray_depth = 1.5;
  request.request.call.occupied_passthrough_threshold = 0;
  
  bool response = current_view_retriever_.call(request);
  
  if( response )
  {
    _output = request.response.expected_information.values;
  }
  
  return response;
}

void ViewPlanner::commandCallback( const std_msgs::StringConstPtr& _msg )
{
  if( _msg->data=="START" )
  {
    start_ = true;
    pause_ = false;
    stop_and_print_ = false;
  }
  else if( _msg->data=="PAUSE" )
  {
    start_ = false;
    pause_ = true;
    stop_and_print_ = false;
  }
  else if( _msg->data=="STOP_AND_PRINT" )
  {
    start_ = false;
    pause_ = false;
    stop_and_print_ = true;
  }
  else if( _msg->data=="REINIT" )
  {
    reinit_ = true;
  }
}


}