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

#include "dense_reconstruction/PlanningSpaceInitializationInfoMsg.h"
#include "dense_reconstruction/ViewMsg.h"
#include "dense_reconstruction/FeasibleViewSpaceRequest.h"
#include "dense_reconstruction/ViewRequest.h"
#include "dense_reconstruction/RetrieveData.h"
#include "dense_reconstruction/MovementCostCalculation.h"
#include "dense_reconstruction/MovementCostMsg.h"
#include "dense_reconstruction/MoveToOrder.h"


namespace dense_reconstruction
{

/** abstract interface class that serves as an interface between the low level, robot specific planner and the view planner:
 * For each interface function apart from "initializePlanningFrame" there must exist an equivalent service call.
 */
class RobotPlanningInterface
{
public:
  class MovementCost;
  class PlanningSpaceInitializationInfo;
  class DataRetrievalModule;
  
  RobotPlanningInterface();
  
  /** returns the name of the global planning frame (currently "dr_origin" for 'dense reconstruction origin) and does all calculations needed in order to set up the tf tree for that frame, e.g. initialize SVO, save transformation from SVO frame (world) to (dr_origin) etc. (TODO load from parameter)
   */
  virtual std::string initializePlanningFrame()=0;
  
  /** initializes the robot's planning space
   * @param _info information the robot needs to setup its planning space
   */
  virtual bool initializePlanningSpace( PlanningSpaceInitializationInfo& _info )=0;
  
  /** returns the current view */
  virtual View getCurrentView()=0;
  
  /** returns the view space that is available for planning (the idea is that it consists exclusively of poses that are considered to be reachable aforehand by the robot, considering its restraints
   * @param _space pointer to the ViewSpace object that should be filled
   * @return false if it failed
   */
  virtual bool getPlanningSpace( ViewSpace* _space )=0;
  
  /** returns a sub space of the view space with all view points available within a certain range _distance of _view
   * @param _space pointer to the ViewSpace object that should be filled
   * @param _view view from which to take the distance
   * @return false if it failed
   */
  //virtual bool getSubPlanningSpace( ViewSpace* _space, View& _view, double _distance )=0;
  
  /** using an enum in order to possibly return more information than just true/false
   */
  enum ReceiveInfo{ RECEIVED, RECEPTION_FAILED };
  
  /** executes the actions needed to retrieve new data, e.g. scanning movements until remode converges
   * @return information about what happened (data received, receival failed )
   */
  virtual ReceiveInfo retrieveData()=0;
  
  /** returns the cost to move from the current view to the indicated view
   * @param _target_view the next view
   * @return cost to move to that view
   */
  virtual MovementCost movementCost( View& _target_view )=0;
  
  /** returns the cost to move from start view to target view
   * @param _start_view the start view
   * @param _target_view the target view
   * @return cost for the movement
   */
  virtual MovementCost movementCost( View& _start_view, View& _target_view )=0;
  
  /** tells the robot to get the camera to a new view
   * @param _target_view where to move to
   * @return false if the operation failed
   */
  virtual bool moveTo( View& _target_view )=0;
  
private:
};

/** class to hold the cost it takes to move along a path */
class RobotPlanningInterface::MovementCost
{
public:
  double cost; // keeping it simple
  /// possible exceptions:: INFINITE_COST: do not move to target view, INVALID_STATE: robot is in state which somehow prevents it from calculating a cost, but the movement might be possible
  enum Exception{ COST_UNKNOWN, INFINITE_COST, INVALID_STATE, INVALID_TARGET_STATE, INVALID_START_STATE };
  Exception exception;
  
  /** converts the cost to a message
   */
  MovementCostMsg toMsg();
  
  /**
   * loads from msg
   */
  void fromMsg( MovementCostMsg& _msg );
};

class RobotPlanningInterface::PlanningSpaceInitializationInfo
{
public:
  class RobotSpaceInfo;
  
  boost::shared_ptr<RobotSpaceInfo> getSpecifics();
  
  /**
   * template for setting the specifics, attempts to type cast
   */
  template<class T>
  void setSpecifics( boost::shared_ptr<T> _specifics );
private:
  boost::shared_ptr<RobotSpaceInfo> robot_specific_info_;
};

class RobotPlanningInterface::PlanningSpaceInitializationInfo::RobotSpaceInfo
{
public:
  virtual std::string type()=0;
  
};

/// module for retrieving data, encapsulates the interface to the stereo reconstruction module (e.g. REMODE)
class RobotPlanningInterface::DataRetrievalModule
{
public:
  /**
   * attempts to retrieve data and reports on success, call is blocking
   */
  virtual ReceiveInfo retrieveData()=0;
};

template<class T>
void RobotPlanningInterface::PlanningSpaceInitializationInfo::setSpecifics( boost::shared_ptr<T> _specifics )
{
  robot_specific_info_ = boost::dynamic_pointer_cast<RobotSpaceInfo>(_specifics);
}

}