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
  
  // octomap information service test
  
  dense_reconstruction::YoubotPlanner youbot(&n);
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
  
  using namespace dense_reconstruction;
  std::vector<View, Eigen::aligned_allocator<View> > view_space_copy = view_space.getViewSpace();
  
  
  ViewInformationReturn service;
  service.request.call.poses.push_back( movements::toROS( view_space_copy[0].pose()) );
  service.request.call.ray_resolution_x = 0.2;
  service.request.call.ray_resolution_y = 0.2;
  service.request.call.min_ray_depth = 0.05;
  service.request.call.max_ray_depth = 1.5;
  service.request.call.occupied_passthrough_threshold = 0;
  service.request.call.metric_names.push_back("NrOfUnknownVoxels");
  
  while(true)
  {
    char egal;
    std::cin>>egal; // just get time to shut down gazebo and start up octomap and the bag
    
    if( !ros::service::call("/dense_reconstruction/octomap/information", service) )
    {
      ROS_INFO("Server call failed");
    }
    else
    {
      for(int i=0;i<service.response.expected_information.metric_names.size();++i)
      {
	using namespace std;
	cout<<service.response.expected_information.metric_names[i]<<": "<<service.response.expected_information.values[i]<<endl;
      }
    }
  }
  
  return 0;
  
  /*
  // YoubotPlanner tests
  dense_reconstruction::YoubotPlanner youbot(&n);
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
  
  using namespace dense_reconstruction;
   std::vector<View, Eigen::aligned_allocator<View> > view_space_copy = view_space.getViewSpace();
  
  
  
  dense_reconstruction::View closest = view_space.getAClosestNeighbour(current_view);
  
  /*
  boost::shared_ptr<dense_reconstruction::View::ViewInfo> info_reference = closest.associatedData();
  boost::shared_ptr<YoubotPlanner::ViewInfo> info = boost::dynamic_pointer_cast<YoubotPlanner::ViewInfo>(info_reference);
  YoubotPlanner::ViewPointData* data = info->getViewPointData();
  ROS_ERROR_STREAM("data in main link1 angle is: "<<data->link1_config_.angle_);*/
  /*
  ROS_INFO("Initial scannning");
  youbot.retrieveData();
  ROS_INFO("To view 1");
  ROS_INFO_STREAM("Cost is: "<<youbot.movementCost(view_space_copy[0]).cost );
  youbot.moveTo(view_space_copy[0]);  
  youbot.retrieveData();
  ROS_INFO("To view 2");
  ROS_INFO_STREAM("Cost is: "<<youbot.movementCost(view_space_copy[1]).cost );
  youbot.moveTo(view_space_copy[1]);  
  youbot.retrieveData();
  ROS_INFO("To view 3");
  ROS_INFO_STREAM("Cost is: "<<youbot.movementCost(view_space_copy[2]).cost );
  youbot.moveTo(view_space_copy[2]);  
  youbot.retrieveData();
  ROS_INFO("To view 4");
  ROS_INFO_STREAM("Cost is: "<<youbot.movementCost(view_space_copy[3]).cost );
  youbot.moveTo(view_space_copy[3]);  
  youbot.retrieveData();
  ROS_INFO("To view 5");
  ROS_INFO_STREAM("Cost is: "<<youbot.movementCost(view_space_copy[4]).cost );
  youbot.moveTo(view_space_copy[4]);  
  youbot.retrieveData();
  ROS_INFO("To view 6");
  ROS_INFO_STREAM("Cost is: "<<youbot.movementCost(view_space_copy[5]).cost );
  youbot.moveTo(view_space_copy[5]);  
  youbot.retrieveData();
  
  
  ROS_INFO("If the program terminates now it has reached the correct exit point");
  ros::shutdown();
  return 0;
  
  
  
  
  
  
  std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > joint_values;
  std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > grid;
  youbot.calculateArmGrid( 60, 60, joint_values, &grid );
  
  std::ofstream out("/home/stewess/Documents/youbot_arm_grid_50pts_per_m.txt", std::ofstream::trunc);
  BOOST_FOREACH( auto point, grid )
  {
    out<<point(0)<<" "<<point(1)<<"\n";
  }
  out.close();
  
  std::ofstream out2("/home/stewess/Documents/youbot_arm_joint_values_50pts_per_m.txt", std::ofstream::trunc);
  BOOST_FOREACH( auto value, joint_values )
  {
    out2<<value(0)<<" "<<value(1)<<" "<<value(2)<<"\n";
  }
  out2.close();
  
  return 0;*/
} 
