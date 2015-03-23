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
{
  
  view_space_retriever_ = nh_.serviceClient<dense_reconstruction::FeasibleViewSpaceRequest>("/dense_reconstruction/robot_interface/feasible_view_space");
  current_view_retriever_ = nh_.serviceClient<dense_reconstruction::ViewRequest>("/dense_reconstruction/robot_interface/current_view");
  data_retriever_ = nh_.serviceClient<dense_reconstruction::RetrieveData>("/dense_reconstruction/robot_interface/retrieve_data");
  cost_retriever_ = nh_.serviceClient<dense_reconstruction::MovementCostCalculation>("/dense_reconstruction/robot_interface/movement_cost");
  view_information_retriever_ = nh_.serviceClient<dense_reconstruction::ViewInformationReturn>("/dense_reconstruction/3d_model/information");
  robot_mover_ = nh_.serviceClient<dense_reconstruction::MoveToOrder>("/dense_reconstruction/robot_interface/move_to");
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

bool ViewPlanner::getViewInformation( std::vector<double>& _output, movements::PoseVector& _poses, std::vector<std::string>& _metric_names )
{
  ViewInformationReturn request;
  request.request.call.poses = movements::toROS(_poses);
  request.request.call.metric_names = _metric_names;
  
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


}