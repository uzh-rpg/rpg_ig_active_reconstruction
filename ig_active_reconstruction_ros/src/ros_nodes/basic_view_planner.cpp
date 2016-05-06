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

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ig_active_reconstruction/basic_view_planner.hpp>
#include <ig_active_reconstruction/weighted_linear_utility.hpp>
#include <ig_active_reconstruction/max_calls_termination_criteria.hpp>

#include "ig_active_reconstruction_ros/param_loader.hpp"
#include "ig_active_reconstruction_ros/robot_ros_client_ci.hpp"
#include "ig_active_reconstruction_ros/views_ros_client_ci.hpp"
#include "ig_active_reconstruction_ros/world_representation_ros_client_ci.hpp"

/*! Implements a ROS node holding a BasicViewPlanner, combined with a simple command line user interface.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_view_planner");
  ros::NodeHandle nh;
  
  namespace iar = ig_active_reconstruction;
  
  
  
  
  // load parameter configuration
  // ...................................................................................................................
  
  // for the view planner:
  iar::BasicViewPlanner::Config bvp_config;
  ros_tools::getParam( bvp_config.discard_visited, "discard_visited", false );
  ros_tools::getParam( bvp_config.max_visits, "max_visits", -1 );
  
  // for the utility calculator
  double cost_weight;
  ros_tools::getParam( cost_weight, "cost_weight", 1.0 );
  
  // for the termination critera
  unsigned int max_calls;
  ros_tools::getParam<unsigned int, int>( max_calls, "max_calls", 20 );
  
  
  
  // only the view planner resides here
  // ...................................................................................................................
  iar::BasicViewPlanner view_planner(bvp_config);
  
  
  // robot, viewspace module and world representation are external
  // ...................................................................................................................
  std::shared_ptr<iar::robot::CommunicationInterface> robot_comm = std::make_shared<iar::robot::RosClientCI>(nh);
  std::shared_ptr<iar::views::CommunicationInterface> views_comm = std::make_shared<iar::views::RosClientCI>(nh);
  std::shared_ptr<iar::world_representation::CommunicationInterface> world_comm = std::make_shared<iar::world_representation::RosClientCI>(nh);
  
  view_planner.setRobotCommUnit(robot_comm);
  view_planner.setViewsCommUnit(views_comm);
  view_planner.setWorldCommUnit(world_comm);
  
  
  // want to use the weighted linear utility calculator, which directly interacts with world and robot comms too
  // ...................................................................................................................
  std::shared_ptr<iar::WeightedLinearUtility> utility_calculator = std::make_shared<iar::WeightedLinearUtility>(cost_weight);
  utility_calculator->setRobotCommUnit(robot_comm);
  utility_calculator->setWorldCommUnit(world_comm);
  
  view_planner.setUtility(utility_calculator);
  
  
  // using a simple max. number of calls termination critera
  // ...................................................................................................................
  std::shared_ptr<iar::GoalEvaluationModule> termination_criteria = std::make_shared<iar::MaxCallsTerminationCriteria>(max_calls);
  
  view_planner.setGoalEvaluationModule(termination_criteria);
  
  
  
  
  
  
  
  // Simple command line user interface.
  // ...................................................................................................................
  ROS_INFO("Basic View Planner was successfully setup. As soon as other modules are running, we're ready to go.");
  
  std::string gui_info = "\n\n\nBASIC VIEW PLANNER SIMPLE UI\n********************************\nThe following actions are supported ('key toggle'):\n- 'g' (go) Start or unpause view planning.\n- 'p': (pause) Pause procedure.\n- 's' (stop) Stop procedure\n- 'q' (quit) Stop procedure and quit program.\n\n";
  char user_input;
  
  while(true)
  {
    std::cout<<gui_info;
    std::cin>>user_input;
    
    switch(user_input)
    {
      case 'g':
	std::cout<<"Starting...";
	view_planner.run();
	break;
      case 'p':
	std::cout<<"Pausing...";
	view_planner.pause();
	break;
      case 's':
	while(true)
	{
	  std::cout<<"\n\nAre you sure you want to stop the procedure? (y/n)\n";
	  std::cin>>user_input;
	  if(user_input=='y')
	  {
	    view_planner.stop();
	    break;
	  }
	  else if(user_input=='n')
	    break;
	};
	
	break;
      case 'q':
	while(true)
	{
	  std::cout<<"\n\nAre you sure you want to quit the program? (y/n)\n";
	  std::cin>>user_input;
	  if(user_input=='y')
	  {
	    view_planner.stop();
	    return 0;
	  }
	  else if(user_input=='n')
	    break;
	};
	
	break;
    };
  }
  
  return 0;
}