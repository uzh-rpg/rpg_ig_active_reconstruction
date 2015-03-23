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

#pragma once


#include "ros/ros.h"
#include <movements/core>
#include <movements/ros_movements.h>

#include "dense_reconstruction/robot_planning_interface.h"
#include "dense_reconstruction/ViewInformationReturn.h"
#include "std_msgs/String.h"


namespace dense_reconstruction
{

/** simple, dynamic programming based next best view planner for dense reconstruction
 */
class ViewPlanner
{
public:
  ViewPlanner( ros::NodeHandle& _n );
  
  /**
   * starts the view planner, it will however only start taking actions when it gets the command on the "dense_reconstruction/view_planner/command" topic
   */
  void run();
  
  /**
   * waits and spins once
   */
  void waitAndSpin( double _sec=0.5 );
  
  /**
   * to calculate a subset, e.g. to limit the range in which the next view is sought, so far it only filters out already visited or bad positions
   */
  void determineAvailableViewSpace( std::vector<unsigned int>& _output );
  
  /** attempts to load the view space from service
   * returns true if successful
   */
  bool getViewSpace();
  
  /**
   * retrieves the current view by calling the service
   */
  bool getCurrentView( View& _output);
  
  /**
   * retrieves data by calling the surface
   */
  bool retrieveData( RobotPlanningInterface::ReceiveInfo& _output );
  
  /**
   * gets the movement cost for given start- and endpoints
   * returns true if calling the service was successful
   */
  bool movementCost( RobotPlanningInterface::MovementCost& _output, View& _start_view, View& _target_view );
  
  /**
   * calls the service to move the robot somewhere
   * Returns true if calling the service was successful
   */
  bool moveTo( bool& _output, View& _target_view );
  
  /**
   * retreives expected informations for given poses (if more than one then it's a path into the future)
   * @param _output the calculated information  values for the given metrics
   * @param _poses the last pose is the one for which the information is sought, optional other poses describe a path into the future before the sought view information
   * @param _metric_names set of metrics to use to calculate the information
   * @return true if the view information service was called successfully
   */
  bool getViewInformation( std::vector<double>& _output, movements::PoseVector& _poses );
  
  void commandCallback( const std_msgs::StringConstPtr& _msg );
private:
  std::string planning_frame_;
  std::vector<std::string> metrics_to_use_;
  
  ViewSpace view_space_;
  
  View current_view_;
  
  bool start_;
  bool pause_;
  bool stop_and_print_;
  bool reinit_;
  
  std::vector< std::vector<double> > planning_data_; /// container for data gathered during planning: visited views, informations and costs
  
  ros::NodeHandle nh_;
  ros::Subscriber command_;
  
  ros::ServiceClient view_space_retriever_;
  ros::ServiceClient current_view_retriever_;
  ros::ServiceClient data_retriever_;
  ros::ServiceClient cost_retriever_;
  ros::ServiceClient view_information_retriever_;
  ros::ServiceClient robot_mover_;
};

}