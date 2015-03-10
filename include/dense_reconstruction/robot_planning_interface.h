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

#include "dense_reconstruction/view_space.h"
#include "dense_reconstruction/view.h"


namespace dense_reconstruction
{

/** abstract interface class that serves as an interface between the low level, robot specific planner and the view planner
 */
class RobotPlanningInterface
{
public:
  class MovementCost;
  
  RobotPlanningInterface();
  
  /** returns the name of the global planning frame (currently "dr_origin" for 'dense reconstruction origin) and does all calculations needed in order to set up the tf tree for that frame, e.g. initialize SVO, save transformation from SVO frame (world) to (dr_origin) etc.
   */
  std::string initializePlanningFrame()=0;
  
  /** returns the view space that is available for planning (the idea is that it consists exclusively of poses that are considered to be reachable aforehand by the robot, considering its restraints
   * @param _space pointer to the ViewSpace object that should be filled
   * @return false if it failed
   */
  bool getPlanningSpace( ViewSpace* _space )=0;
  
  /** using an enum in order to possibly return more information than just true/false
   */
  enum ReceiveInfo{ RECEIVED, RECEPTION_FAILED };
  
  /** executes the actions needed to retrieve new data, e.g. scanning movements until remode converges
   * @return information about what happened (data received, receival failed )
   */
  ReceiveInfo retrieveData()=0;
  
  /** returns the cost to move from the current view to the indicated view
   * @param _target_view the next view
   * @return cost to move to that view
   */
  MovementCost calculateCost( View& _target_view )=0;
  
  /** tells the robot to get the camera to a new view
   * @param _target_view where to move to
   * @return false if the operation failed
   */
  bool moveTo( View& _target_view )=0;
  
private:
};

/** class to hold the cost it takes to move along a path */
class RobotPlanningInterface::MovementCost
{
public:
  double cost; // keeping it simple
};

}