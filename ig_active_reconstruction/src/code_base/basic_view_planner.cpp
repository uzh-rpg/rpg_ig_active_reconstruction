/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of ig_active_reconstruction, a ROS package for...well,

ig_active_reconstruction is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
ig_active_reconstruction is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with ig_active_reconstruction. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ig_active_reconstruction/basic_view_planner.hpp"

#include <chrono>

namespace ig_active_reconstruction
{
  
  BasicViewPlanner::Config::Config()
  : discard_visited(false)
  , max_visits(-1)
  {
  }
  
  BasicViewPlanner::BasicViewPlanner(Config config)
  : config_(config)
  , robot_comm_unit_(nullptr)
  , views_comm_unit_(nullptr)
  , world_comm_unit_(nullptr)
  , utility_calculator_(nullptr)
  , goal_evaluation_module_(nullptr)
  , status_(Status::UNINITIALIZED)
  , runProcedure_(false)
  , pauseProcedure_(false)
  {
    
  }
  
  BasicViewPlanner::~BasicViewPlanner()
  {
    runProcedure_ = false;
    if( running_procedure_.joinable() )
      running_procedure_.join();
  }
  
  void BasicViewPlanner::setRobotCommUnit( std::shared_ptr<robot::CommunicationInterface> robot_comm_unit )
  {
    if( runProcedure_ || running_procedure_.joinable() )
      return;
    
    robot_comm_unit_ = robot_comm_unit;
    
    if( isReady() && status_==Status::UNINITIALIZED )
      status_ = Status::IDLE;
    else
      status_ = Status::UNINITIALIZED;
  }
  
  void BasicViewPlanner::setViewsCommUnit( std::shared_ptr<views::CommunicationInterface> views_comm_unit )
  {
    if( runProcedure_ || running_procedure_.joinable() )
      return;
    
    views_comm_unit_ = views_comm_unit;
    
    if( isReady() && status_==Status::UNINITIALIZED )
      status_ = Status::IDLE;
    else
      status_ = Status::UNINITIALIZED;
  }
  
  void BasicViewPlanner::setWorldCommUnit( std::shared_ptr<world_representation::CommunicationInterface> world_comm_unit )
  {
    if( runProcedure_ || running_procedure_.joinable() )
      return;
    
    world_comm_unit_ = world_comm_unit;
    
    if( isReady() && status_==Status::UNINITIALIZED )
      status_ = Status::IDLE;
    else
      status_ = Status::UNINITIALIZED;
  }
  
  void BasicViewPlanner::setUtility( std::shared_ptr<UtilityCalculator> utility_calculator )
  {
    if( runProcedure_ || running_procedure_.joinable() )
      return;
    
    utility_calculator_ = utility_calculator;
    
    if( isReady() && status_==Status::UNINITIALIZED )
      status_ = Status::IDLE;
    else
      status_ = Status::UNINITIALIZED;
  }
  
  void BasicViewPlanner::setGoalEvaluationModule( std::shared_ptr<GoalEvaluationModule> goal_evaluation_module )
  {
    if( runProcedure_ || running_procedure_.joinable() )
      return;
    
    goal_evaluation_module_ = goal_evaluation_module;
    
    if( isReady() && status_==Status::UNINITIALIZED )
      status_ = Status::IDLE;
    else
      status_ = Status::UNINITIALIZED;
  }
  
  bool BasicViewPlanner::run()
  {
    if( runProcedure_ || running_procedure_.joinable() )
    {
      if( pauseProcedure_ )
      {
	pauseProcedure_ = false;
	return true;
      }
      
      return false;
    }
    
    runProcedure_ = true;
    running_procedure_ = std::thread(&BasicViewPlanner::main, this);
    
    return true;
  }
  
  void BasicViewPlanner::pause()
  {
    pauseProcedure_ = true;
  }
  
  void BasicViewPlanner::stop()
  {
    runProcedure_ = false;
  }
  
  BasicViewPlanner::Status BasicViewPlanner::status()
  {
    return status_;
  }
  
  bool BasicViewPlanner::isReady()
  {
    return robot_comm_unit_!=nullptr
	&& views_comm_unit_!=nullptr
	&& world_comm_unit_!=nullptr
	&& utility_calculator_!=nullptr
	&& goal_evaluation_module_!=nullptr;
  }
  
  void BasicViewPlanner::main()
  {    
    // preparation
    goal_evaluation_module_->reset();
    
    // get viewspace................................................
    viewspace_ = std::make_shared<views::ViewSpace>();
    views::CommunicationInterface::ViewSpaceStatus viewspace_status;
    do
    {
      status_ = Status::DEMANDING_VIEWSPACE;
      viewspace_status = views_comm_unit_->getPlanningSpace( viewspace_.get() );
      
      if( !runProcedure_ ) // exit point
	{
	  status_ = Status::IDLE;
	  runProcedure_ = false;
	  return;
	}
      pausePoint();
      
    }while( viewspace_status!=views::CommunicationInterface::ViewSpaceStatus::OK );
      
    do
    {
      // determine view candidate subset of viewspace .....................
      views::ViewSpace::IdSet view_candidate_ids;
      viewspace_->getGoodViewSpace(view_candidate_ids, config_.discard_visited);
      
      if(view_candidate_ids.empty())
      {
	break;
      }
      
      // receive data....................................................
      robot::CommunicationInterface::ReceptionInfo data_retrieval_status;
      do
      {
	status_ = Status::DEMANDING_NEW_DATA;
	data_retrieval_status = robot_comm_unit_->retrieveData();
	
	if( !runProcedure_ ) // exit point
	{
	  status_ = Status::IDLE;
	  runProcedure_ = false;
	  return;
	}
	pausePoint();
	
      }while( data_retrieval_status != robot::CommunicationInterface::ReceptionInfo::SUCCEEDED );
      
      
      // getting cost and ig is wrapped in the utility calculator..................
      status_ = Status::NBV_CALCULATIONS;
      views::View::IdType nbv_id = utility_calculator_->getNbv(view_candidate_ids,viewspace_);
      views::View nbv = viewspace_->getView(nbv_id);
      
      // check termination criteria ...............................................
      if( goal_evaluation_module_->isDone() )
	break;
      
      // move to next best view....................................................
      bool successfully_moved = false;
      do
      {
	status_ = Status::DEMANDING_MOVE;
	robot_comm_unit_->moveTo(nbv);
	
	if( !runProcedure_ ) // exit point
	{
	  status_ = Status::IDLE;
	  runProcedure_ = false;
	  return;
	}
	pausePoint();
	
      }while(!successfully_moved);
      
      // update viewspace
      viewspace_->setVisited(nbv_id);
      if( config_.max_visits!=-1 && viewspace_->timesVisited(nbv_id) >= config_.max_visits )
	viewspace_->setBad(nbv_id);
      
    }while( runProcedure_ );
    
    status_ = Status::IDLE;
    runProcedure_ = false;
    return;
  }
  
  void BasicViewPlanner::pausePoint()
  {
    while(pauseProcedure_)
    {
      status_ = Status::PAUSED;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}