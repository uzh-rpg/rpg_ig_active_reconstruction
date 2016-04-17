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
#include "dense_reconstruction/view_space.h"
#include "dense_reconstruction/robot_planning_interface.h"

namespace dense_reconstruction
{
  
/** container for costs
  */
class CostMatrix
{
public:
  CostMatrix();
  
  /** creates a cost matrix for the passed view space
   */
  CostMatrix( ViewSpace& _view_space );
  
  /**
   * gets the movement cost for given start- and endpoints
   * returns true if calling the service was successful
   */
  bool movementCost( RobotPlanningInterface::MovementCost& _output, View& _start_view, View& _target_view );
  
  /** creates a cost matrix for the passed view space
   */
  bool createCostPairs( ViewSpace& _view_space );
  
  /** saves the cost matrix to disk
   * @param _path name and path of the file to be written
   */
  void toFile( std::string _path_name );
  
private:
  ros::ServiceClient cost_retriever_;
  std::map<int,map<int,double> > cost_matrix_;
  unsigned int view_space_size_;
};
  
}