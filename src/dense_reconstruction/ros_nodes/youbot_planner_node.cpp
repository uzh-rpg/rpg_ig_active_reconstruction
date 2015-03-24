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

 
#include "dense_reconstruction/youbot_planning.h"
#include "dense_reconstruction/robot_planning_interface.h"
#include "dense_reconstruction/ViewInformationReturn.h"
#include "boost/foreach.hpp"

#include <random>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "youbot_reconstruction_controller");
  ros::NodeHandle n;
  
  // YoubotPlanner tests
  dense_reconstruction::YoubotPlanner youbot(&n);
  
  ros::spin();
  /*
  youbot.initializePlanningFrame();
  
  dense_reconstruction::RobotPlanningInterface::PlanningSpaceInitializationInfo simple_setup;
  boost::shared_ptr<dense_reconstruction::YoubotPlanner::SpaceInfo> specifics( new dense_reconstruction::YoubotPlanner::SpaceInfo() );
  
  specifics->approximate_relative_object_position_ << 1,0,0;
  specifics->base_pts_per_circle_ = 2;
  specifics->arm_min_view_distance_ = 0.3;
  specifics->arm_view_resolution_ = 100; // [pts/m]
  
  simple_setup.setSpecifics(specifics);
  
  youbot.initializePlanningSpace(simple_setup);
  
  dense_reconstruction::ViewSpace view_space;
  youbot.getPlanningSpace( &view_space );
    
  dense_reconstruction::View current_view = youbot.getCurrentView();
  */
  ros::shutdown();
  return 0;
  
} 
