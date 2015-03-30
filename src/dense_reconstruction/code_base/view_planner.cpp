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
#include "boost/foreach.hpp"
#include "utils/math.h"
#include <fstream>
#include <sstream>
#include <list>
#include <ctime>


namespace dense_reconstruction
{

ViewPlanner::ViewPlanner( ros::NodeHandle& _n )
  :nh_(_n)
  ,start_(false)
  ,pause_(false)
  ,stop_and_print_(false)
  ,reinit_(false)
  ,abort_loop_(false)
  ,max_movement_attempts_(3)
  ,mark_visited_(true)
{
  
  if( !nh_.getParam("/view_planner/data_folder", data_folder_) )
  {
    ROS_WARN("No data folder was found on parameter server. Planning data will be saved to ROS execution directory...");
  }
  
  if( !nh_.getParam("/view_planner/discard_visited", mark_visited_) )
  {
    ROS_WARN("You didn't specify whether to discard already visited view or not. (No '/view_planner/discard_visited' param found on server), by default visited views are discarded.");
  }
  
  if( !nh_.getParam("/view_planner/cost_weight", cost_weight_) )
  {
    ROS_WARN("No cost weight was found on parameter server. default '1.0' will be used...");
    cost_weight_ = 1.0;
  }
  if( !nh_.getParam("/view_planner/information_weight", information_weight_) )
  {
    ROS_WARN("No cost weight was found on parameter server. default '1.0' will be used...");
    information_weight_ = 1.0;
  }
  
  if( !nh_.getParam("/view_planner/information/ray_resolution_x", ray_resolution_x_) )
  {
    ROS_WARN("Undefined parameter '/view_planner/information/ray_resolution_x'. Default '0.1' will be used...");
    ray_resolution_x_ = 0.1;
  }
  if( !nh_.getParam("/view_planner/information/ray_resolution_y", ray_resolution_y_) )
  {
    ROS_WARN("Undefined parameter '/view_planner/information/ray_resolution_y'. Default '0.1' will be used...");
    ray_resolution_y_ = 0.1;
  }
  if( !nh_.getParam("/view_planner/information/ray_step_size", ray_step_size_) )
  {
    ROS_WARN("Undefined parameter '/view_planner/information/ray_step_size'. Default '2' will be used...");
    ray_step_size_ = 2;
  }
  if( !nh_.getParam("/view_planner/information/subwindow_width_percentage", subwindow_width_percentage_) )
  {
    ROS_WARN("Undefined parameter '/view_planner/information/subwindow_width_percentage'. Default '0.5' will be used...");
    subwindow_width_percentage_ = 0.5;
  }
  if( !nh_.getParam("/view_planner/information/subwindow_height_percentage", subwindow_height_percentage_) )
  {
    ROS_WARN("Undefined parameter '/view_planner/information/subwindow_height_percentage'. Default '0.5' will be used...");
    subwindow_height_percentage_ = 0.5;
  }
  if( !nh_.getParam("/view_planner/information/min_ray_depth", min_ray_depth_) )
  {
    ROS_WARN("Undefined parameter '/view_planner/information/min_ray_depth'. Default '0.05' will be used...");
    min_ray_depth_ = 0.05;
  }
  if( !nh_.getParam("/view_planner/information/max_ray_depth", max_ray_depth_) )
  {
    ROS_WARN("Undefined parameter '/view_planner/information/max_ray_depth'. Default '1.5' will be used...");
    max_ray_depth_ = 1.5;
  }
  if( !nh_.getParam("/view_planner/information/occupied_passthrough_threshold", occupied_passthrough_threshold_) )
  {
    ROS_WARN("Undefined parameter '/view_planner/information/occupied_passthrough_threshold'. Default '0' will be used...");
    occupied_passthrough_threshold_ = 0;
  }
  
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
  
  BOOST_FOREACH( auto metric_name, metrics_to_use_ )
  {
    double weight=0;
    if( !nh_.getParam("/view_planner/information_metric/"+metric_name+"/weight", weight) )
    {
      ROS_WARN_STREAM("No weight found on parameter server for "<<metric_name<<" metric ("<<"/view_planner/information_metric/"<<metric_name<<"/weight"<<"). Weight will be set to zero and the metric thus not considered in calculations.");
    }
    information_weights_.push_back(weight);
  }
  
  planning_data_names_.push_back("time");
  planning_data_names_.push_back("pos_x");
  planning_data_names_.push_back("pos_y");
  planning_data_names_.push_back("pos_z");
  planning_data_names_.push_back("rot_x");
  planning_data_names_.push_back("rot_y");
  planning_data_names_.push_back("rot_z");
  planning_data_names_.push_back("rot_w");
  planning_data_names_.push_back("return_value");
  planning_data_names_.push_back("winning_margin");
  planning_data_names_.push_back("return_value_mean");
  planning_data_names_.push_back("return_value_stddev");
  planning_data_names_.push_back("cost");
  planning_data_names_.push_back("NrOfUnknownVoxels");
  planning_data_names_.push_back("AverageUncertainty");
  planning_data_names_.push_back("AverageEndPointUncertainty");
  planning_data_names_.push_back("UnknownObjectSideFrontier");
  planning_data_names_.push_back("UnknownObjectVolumeFrontier");
  planning_data_names_.push_back("ClassicFrontier");
  planning_data_names_.push_back("EndNodeOccupancySum");
  planning_data_names_.push_back("TotalOccupancyCertainty");
  planning_data_names_.push_back("TotalNrOfOccupieds");
  
  command_ = nh_.subscribe( "/dense_reconstruction/view_planner/command", 100, &ViewPlanner::commandCallback, this );
  
  save_view_space_server_ = nh_.advertiseService("/dense_reconstruction/view_planner/view_space_to_file", &ViewPlanner::saveViewSpaceToFileService, this );
}

void ViewPlanner::run()
{
  ROS_INFO("Waiting for start signal.");
  while( !start_ ) // wait for start signal
  {
    waitAndSpin();
  }
  ROS_INFO("Starting...");
  
  // get view space from robot_interface
  while( !getViewSpace() )
  {
    ROS_INFO("View space service not available yet. Waiting...");
    waitAndSpin(2);
  }
  
  ROS_INFO_STREAM("The view space has "<<view_space_.size()<<" views.");
  
  // get current view
  while( !getCurrentView(current_view_) )
  {
    ROS_INFO("Attempting to retrieve start view. Waiting...");
    waitAndSpin(2);
  }
  
  ROS_INFO("Successfully received view space and current view.");
  
    
  ROS_INFO("Start planning");
  // enter loop
  do
  {
    pauseIfRequested();
    
    // retrieve new information
    retrieveDataAndWait();
    
    // possibly build subspace of complete space
    std::vector<unsigned int> views_to_consider;
    determineAvailableViewSpace( views_to_consider );
    ROS_INFO_STREAM("Nr of  views that are part of the choice: "<<views_to_consider.size() );
    
    // get movement costs
    ROS_INFO("Retrieve movement costs...");
    std::vector<double> cost(views_to_consider.size());
    std::vector<RobotPlanningInterface::MovementCost> full_cost(views_to_consider.size());
    int achievable_poses_count = 0;
    for( unsigned int i=0; i<cost.size(); i++ )
    {
      RobotPlanningInterface::MovementCost cost_description;
      View target = view_space_.getView( views_to_consider[i] );
      movementCost( cost_description, current_view_, target );
      
      if( cost_description.exception!=RobotPlanningInterface::MovementCost::NONE )
      {
	ROS_WARN_STREAM("Exception occured for movement cost calculation. The flag returned was '"<<cost_description.exception<<"'.");
	view_space_.setBad( views_to_consider[i] ); // don't consider that view
	cost[i] = -1; // no negative costs exist, that's to mark invalids
      }
      else
      {
	cost[i] = cost_description.cost;
	full_cost[i] = cost_description;
	++achievable_poses_count;
      }
    }
    pauseIfRequested();
    
    if( achievable_poses_count==0 )
    {
      ROS_INFO("None of the poses in the set were achievable or the whole view space has been visited. Stopping iteration and saving data gathered so far.");
      //pause_=true;
      //pauseIfRequested();
      
      // reconstruction is done, end iteration
      break;
    }
    ros::spinOnce();
    // get expected informations for each
    ROS_INFO("Retrieve information score...");
    std::vector< std::vector<double> > information(views_to_consider.size());
    for( unsigned int i=0; i<information.size(); i++ )
    {
      ROS_INFO_STREAM("Retrieving information score for view candidate "<<i+1<<"/"<<information.size()<<".");
      
      if( cost[i]==-1 )
	continue;
      
      View target_view = view_space_.getView( views_to_consider[i] );
      
      movements::PoseVector target_positions;
      target_positions.push_back( target_view.pose() );
      
      getViewInformation( information[i], target_positions );
      
      std::cout<<std::endl;
      for( unsigned int j=0; j<metrics_to_use_.size(); ++j )
      {
	ROS_INFO_STREAM(metrics_to_use_[j]<<" score:"<<information[i][j]);
      }
    }
    ros::spinOnce();
    pauseIfRequested();
    
    ROS_INFO("Calculating next best view...");
    // calculate return for each
    std::vector<double> view_returns(views_to_consider.size(),0);
    for( unsigned int i=0; i<view_returns.size(); ++i )
    {
      if( cost[i]==-1 )
	continue;
      
      view_returns[i] = calculateReturn( cost[i], information[i] );
    }
    ros::spinOnce();
    
    
    
    // calculate NBV
    unsigned int nbv_index = 0; // ATTENTION this is the index in the vector here, not the view index in the view space!
    double highest_return = view_returns[0];
    double second_highest = view_returns[0];
    
    std::list<unsigned int> nbv_idx_queue;
    nbv_idx_queue.push_front(0);
    

    for( unsigned int i=0; i<views_to_consider.size(); ++i )
    {
      if( cost[i]==-1 )
	continue;
      
      /*if( view_returns[i]>highest_return )
      {
	nbv_index = i;
	second_highest = highest_return;
	highest_return = view_returns[i];
      }*/
      
      for( std::list<unsigned int>::iterator it = nbv_idx_queue.begin(); it!=nbv_idx_queue.end(); ++it )
      {
	if( view_returns[i] > view_returns[*it] )
	{
	  nbv_idx_queue.insert(it, i );
	  break;
	}
	else if( it==(--nbv_idx_queue.end()) )
	{
	  nbv_idx_queue.insert(nbv_idx_queue.end(), i );
	  break;
	}
      }
    }
    
    ros::spinOnce();
    
    
    // data storage...
    ReturnValueInformation return_info;
    return_info.return_value = view_returns[nbv_idx_queue.front()];
    return_info.winning_margin = view_returns[nbv_idx_queue.front()] - view_returns[*(++nbv_idx_queue.begin())];
    st_is::StdError return_val_errors(view_returns);
    return_info.return_value_mean = return_val_errors.mean;
    return_info.return_value_stddev = std::sqrt(return_val_errors.variance);
    
    saveNBVData(views_to_consider[nbv_idx_queue.front()], return_info, cost[nbv_idx_queue.front()], information[nbv_idx_queue.front()], &full_cost[nbv_idx_queue.front()].additional_field_names, &full_cost[nbv_idx_queue.front()].additional_fields_values );
    
    saveCurrentChoiceDataToFile(views_to_consider, view_returns, cost, information, &full_cost );
    
    ROS_INFO_STREAM(planning_data_.size()<<" data points have been visited and saved so far.");
    
    // check if termination criteria is fulfilled
    if( !terminationCriteriaFulfilled(highest_return, cost[nbv_idx_queue.front()], information[nbv_idx_queue.front()]) )
    {
      // move to this view
      std::list<unsigned int>::iterator nbv_it = nbv_idx_queue.begin();
      View nbv = view_space_.getView(views_to_consider[*nbv_it]);
      bool successful_movement = moveToAndWait(nbv);
      while( !successful_movement )
      {
	ROS_INFO("Choosing next best view instead.");
	view_space_.setUnReachable(views_to_consider[*nbv_it]);
	++nbv_it;
	if( nbv_it==nbv_idx_queue.end() ) // no more view available
	{
	  ROS_INFO("No more views available, terminating...");
	  stop_and_print_ = true;
	  break;
	}
	nbv = view_space_.getView(views_to_consider[*nbv_it]);
	successful_movement = moveToAndWait(nbv);
      }
      
      // set view as visited!
      if( successful_movement && mark_visited_ )
	view_space_.setVisited(views_to_consider[nbv_idx_queue.front()]);
      
    }
    else
    {
      // reconstruction is done, end iteration
      ROS_INFO("The termination critera was fulfilled and the reconstruction is thus considered to have succeeded. The view planner will shut down.");
      break;
    }
    
    ros::spinOnce();
  }while(!stop_and_print_);
  
  ROS_INFO("Saving data to file.");
  saveDataToFile();
}

void ViewPlanner::waitAndSpin(double _sec)
{
  ros::Duration(0.5).sleep();
  ros::spinOnce();
}

void ViewPlanner::pauseIfRequested()
{
  if( pause_ )
  {
    ROS_INFO("Paused.");
    do
    {
      ros::Duration(1.0).sleep();
      ros::spinOnce();
    }while( pause_ );
  }
}

void ViewPlanner::determineAvailableViewSpace( std::vector<unsigned int>& _output )
{
  view_space_.getGoodViewSpace(_output);
}

double ViewPlanner::calculateReturn( double _cost, std::vector<double>& _informations )
{
  double view_return = -1*cost_weight_*_cost;
  
  if( _informations.size()>information_weights_.size() )
  {
    ROS_ERROR_STREAM("ViewPlanner::calculateReturn::Not enough information weights available ("<<information_weights_.size()<<") for the number of information values given ("<<_informations.size()<<") Information is not considered for return value.");
    return view_return;
  }
  
  for( unsigned int i=0; i<_informations.size(); ++i )
  {
    view_return += information_weight_*information_weights_[i]*_informations[i];
  }
  
  return view_return;
}

bool ViewPlanner::terminationCriteriaFulfilled( double _return_value, double _cost, std::vector<double>& _information_gain )
{
  // TODO: need to test which values/metrics saturate and then pick one of these and test it for saturation
  return false;
}

void ViewPlanner::saveNBVData( unsigned int _nbv_index, ReturnValueInformation& _return_value_information, double _cost, std::vector<double>& _information_gain, std::vector<std::string>* _additional_field_names, std::vector<double>* _additional_field_values )
{
  std::vector<double> nbv_data;
  View nbv = view_space_.getView(_nbv_index);
  
  ros::Time now = ros::Time::now();
  
  nbv_data.push_back( now.toSec() );
  nbv_data.push_back( nbv.pose().position.x() );
  nbv_data.push_back( nbv.pose().position.y() );
  nbv_data.push_back( nbv.pose().position.z() );
  nbv_data.push_back( nbv.pose().orientation.x() );
  nbv_data.push_back( nbv.pose().orientation.y() );
  nbv_data.push_back( nbv.pose().orientation.z() );
  nbv_data.push_back( nbv.pose().orientation.w() );
  nbv_data.push_back( _return_value_information.return_value );
  nbv_data.push_back( _return_value_information.winning_margin );
  nbv_data.push_back( _return_value_information.return_value_mean );
  nbv_data.push_back( _return_value_information.return_value_stddev );
  nbv_data.push_back( _cost );
  BOOST_FOREACH( auto information, _information_gain )
  {
    nbv_data.push_back( information );
  }
  
  if( _additional_field_names!=nullptr && _additional_field_values!=nullptr )
  {
    if( _additional_field_names->size()==_additional_field_values->size() )
    {
      for( unsigned int i=0; i<_additional_field_names->size(); ++i )
      {
	unsigned int index = getIndexForAdditionalField((*_additional_field_names)[i]);
	if( nbv_data.size()<=index )
	{
	  nbv_data.resize( index+1 );
	}
	nbv_data[index] = (*_additional_field_values)[i];
      }
    }
  }
  
  planning_data_.push_back(nbv_data);
}

void ViewPlanner::saveDataToFile()
{
  if( planning_data_.size()==0 )
    return;
  
  std::stringstream filename;
  ros::Time now = ros::Time::now();
  time_t current_time;
  time(&current_time);
  filename << data_folder_<<"planning_data"<<current_time<<".data";
  std::string file_name = filename.str();
  
  std::ofstream out(file_name, std::ofstream::trunc);
  
  // print weights
  out<<"cost_weight: "<<cost_weight_<<"\ninformation_weight: "<<information_weight_;
  for( unsigned int i=0; i<metrics_to_use_.size(); ++i )
  {
    out<<"\n"<<metrics_to_use_[i]<<" weight: "<<information_weights_[i];
  }
  
  // then line with names
  out<<"\n"<<planning_data_names_[0];
  for( unsigned int i=1; i<planning_data_names_.size(); ++i )
  {
    out<<" "<<planning_data_names_[i];
  }
  // then all values
  for( unsigned int i=0; i<planning_data_.size(); ++i )
  {
    out<<"\n";
    out<<planning_data_[i][0];
    for( unsigned int j=1; j<planning_data_[i].size(); ++j )
    {
      out<<" "<<planning_data_[i][j];
    }
  }
  out.close();
}

void ViewPlanner::saveCurrentChoiceDataToFile( std::vector<unsigned int> _views_to_consider, std::vector<double> _view_returns, std::vector<double> _costs, std::vector< std::vector<double> >& _information_gain, std::vector<RobotPlanningInterface::MovementCost>* _detailed_costs )
{
  int number_of_views = _views_to_consider.size();
  if( number_of_views==0 )
    return;
  if( number_of_views != _view_returns.size() ||
     number_of_views != _costs.size() ||
     number_of_views != _information_gain.size()
  )
  {
    ROS_ERROR("ViewPlanner::saveCurrentChoiceDataToFile:: Failed because array sizes didn't match.");
    return;
  }
  if( _detailed_costs!=nullptr )
  {
    if( _detailed_costs->size()!=number_of_views)
    {
      ROS_ERROR("ViewPlanner::saveCurrentChoiceDataToFile:: Failed because size of the _detailed_costs array didn't match the others.");
      return;
    }
  }
     
  std::stringstream filename;
  ros::Time now = ros::Time::now();
  time_t current_time;
  time(&current_time);
  filename << data_folder_<<"all_scores_at_distinct_steps/scores"<<current_time<<".data";
  std::string file_name = filename.str();
  
  std::ofstream out(file_name, std::ofstream::trunc);
  
  // print weights
  out<<"cost_weight: "<<cost_weight_<<"\ninformation_weight: "<<information_weight_;
  for( unsigned int i=0; i<metrics_to_use_.size(); ++i )
  {
    out<<"\n"<<metrics_to_use_[i]<<" weight: "<<information_weights_[i];
  }
  out<<"\n\nRetrieved for step: "<<planning_data_.size()<<" (0=first step)";
  
  // then line with names
  out<<"\n"<<planning_data_names_[0];
  for( unsigned int i=1; i<planning_data_names_.size(); ++i )
  {
    out<<" "<<planning_data_names_[i];
  }
  
  // then all values
  for( unsigned int i=0; i<number_of_views; ++i )
  {
    std::vector<double> nbv_data;
    View nbv = view_space_.getView(_views_to_consider[i]);
    
    nbv_data.push_back( now.toSec() );
    nbv_data.push_back( nbv.pose().position.x() );
    nbv_data.push_back( nbv.pose().position.y() );
    nbv_data.push_back( nbv.pose().position.z() );
    nbv_data.push_back( nbv.pose().orientation.x() );
    nbv_data.push_back( nbv.pose().orientation.y() );
    nbv_data.push_back( nbv.pose().orientation.z() );
    nbv_data.push_back( nbv.pose().orientation.w() );
    nbv_data.push_back( _view_returns[i] );
    nbv_data.push_back( 0 );
    nbv_data.push_back( 0 );
    nbv_data.push_back( 0 );
    nbv_data.push_back( _costs[i] );
    BOOST_FOREACH( auto information, _information_gain[i] )
    {
      nbv_data.push_back( information );
    }
    
    if( _detailed_costs!=nullptr )
    {
      if( (*_detailed_costs)[i].additional_field_names.size()==(*_detailed_costs)[i].additional_fields_values.size() )
      {
	std::vector<double> sorted_costs;
	unsigned int lowest_additional_field_index=nbv_data.size();
	for( unsigned int j=0; j<(*_detailed_costs)[i].additional_field_names.size(); ++j )
	{
	  unsigned int index = getIndexForAdditionalField((*_detailed_costs)[i].additional_field_names[j]);
	  if( index<lowest_additional_field_index )
	    lowest_additional_field_index=index;
	  if( sorted_costs.size()<=index )
	  {
	    sorted_costs.resize( index+1 );
	  }
	  sorted_costs[index] = ((*_detailed_costs)[i].additional_fields_values)[j];
	}
	for( unsigned int j=lowest_additional_field_index; j<sorted_costs.size(); ++j )
	{
	  nbv_data.push_back(sorted_costs[j]);
	}
      }
    }
    
    out<<"\n";
    out<<ros::Time::now().toSec();
    
    for( unsigned int j=1; j<nbv_data.size(); ++j )
    {
      out<<" "<<nbv_data[j];
    }
  }
  out.close();
}

void ViewPlanner::retrieveDataAndWait( double _sec )
{
  ROS_INFO("Attempting to retrieve data.");
  RobotPlanningInterface::ReceiveInfo receive_info;
  bool receive_service_succeeded = false;
  do
  {
    receive_service_succeeded = retrieveData(receive_info);
    if( !receive_service_succeeded || receive_info!=RobotPlanningInterface::RECEIVED )
    {
      ROS_INFO("retrieveDataAndWait: data retrieval service did not succeed. Trying again in a few...");
      ros::Duration(_sec).sleep();
      ros::spinOnce();
    }
  }while( (!receive_service_succeeded || receive_info!=RobotPlanningInterface::RECEIVED) && !abort_loop_ );
  
  if( abort_loop_ )
  {
    ROS_INFO("retrieveDataAndWait received loop abortion request and stops trying to get the data retrieval service to succeed. The service might have failed.");
    abort_loop_ = false;
  }
  else
  {
    ROS_INFO("Data retrieval service reported successful data retrieval.");
  }
}

bool ViewPlanner::moveToAndWait( View& _target_view, double _sec )
{
  bool successfully_moved;
  bool service_succeeded = false;
  unsigned int attempts = 0;
  do
  {
    ++attempts;
    
    service_succeeded = moveTo( successfully_moved, _target_view );
    
    if( !service_succeeded || !successfully_moved )
    {
      if( attempts==max_movement_attempts_ )
      {
	ROS_INFO_STREAM("moveToAndWait: The service failed or the robot movement has failed "<<attempts<<" times and thus run out of attempts. The commanded view will be marked as unreachable.");
	return false;
      }
      
      ROS_INFO("moveToAndWait: Either the service failed or the robot movement. Trying again in a few...");
      ros::Duration(_sec).sleep();
      ros::spinOnce();
    }
  }while( (!service_succeeded || !successfully_moved) && !abort_loop_ );
  
  if( abort_loop_ )
  {
    ROS_INFO("moveToAndWait received loop abortion request and stops trying to get the robot movement service to succeed. The service might have failed.");
    abort_loop_ = false;
  }
  else
  {
    ROS_INFO("Robot movement service reported successful movement.");
  }
  return true;
}

unsigned int ViewPlanner::getIndexForAdditionalField( std::string _name )
{
  for( unsigned int i=0; i<planning_data_names_.size(); ++i )
  {
    if( planning_data_names_[i]==_name )
      return i;
  }
  
  planning_data_names_.push_back(_name);
  return planning_data_names_.size()-1;
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
  request.request.additional_information = true;
  
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
  
  bool response = robot_mover_.call(request);
  
  if( response )
  {
    _output = request.response.success;
  }
  
  return response;
}

bool ViewPlanner::getViewInformation( std::vector<double>& _output, movements::PoseVector& _poses )
{
  ViewInformationReturn request;
  request.request.call.poses.resize(1);
  request.request.call.poses[0] = movements::toROS( _poses[0] );//movements::toROS(_poses);
  request.request.call.metric_names = metrics_to_use_;
  
  request.request.call.ray_resolution_x = ray_resolution_x_;
  request.request.call.ray_resolution_y = ray_resolution_y_;
  request.request.call.ray_step_size = ray_step_size_;
  
  double image_width = 752;
  double image_height = 480;
  double subwindow_width = subwindow_width_percentage_*image_width; // [px]
  double subwindow_height = subwindow_height_percentage_*image_height; // [px]
  request.request.call.max_x = 376 + subwindow_width/2;
  request.request.call.min_x = 376 - subwindow_width/2;
  request.request.call.max_y = 240 + subwindow_height/2;
  request.request.call.min_y = 240 - subwindow_height/2;
  
  request.request.call.min_ray_depth = min_ray_depth_;
  request.request.call.max_ray_depth = max_ray_depth_;
  request.request.call.occupied_passthrough_threshold = occupied_passthrough_threshold_;
  
  bool response = view_information_retriever_.call(request);
  
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
    ROS_INFO("Received 'START' signal.");
    start_ = true;
    pause_ = false;
    stop_and_print_ = false;
  }
  else if( _msg->data=="PAUSE" )
  {
    ROS_INFO("Received 'PAUSE' signal.");
    start_ = false;
    pause_ = true;
    stop_and_print_ = false;
  }
  else if( _msg->data=="STOP_AND_PRINT" )
  {
    ROS_INFO("Received 'STOP_AND_PRINT' signal.");
    start_ = false;
    pause_ = false;
    stop_and_print_ = true;
  }
  else if( _msg->data=="REINIT" )
  {
    ROS_INFO("Received 'REINIT' signal.");
    reinit_ = true;
  }
  else if( _msg->data=="ABORT_LOOP" )
  {
    ROS_INFO("Received 'ABORT_LOOP' signal.");
    abort_loop_ = true;
  }
  else if( _msg->data=="PRINT_DATA" )
  {
    ROS_INFO("Received 'PRINT_DATA' signal.");
    saveDataToFile();
  }
}

bool ViewPlanner::saveViewSpaceToFileService( SaveViewSpace::Request& _req, SaveViewSpace::Response& _res )
{
  view_space_.saveToFile( _req.save_path );
  return true;
}

}