/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of ig_based_active_reconstruction, a ROS package for...well,

ig_based_active_reconstruction is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
ig_based_active_reconstruction is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with ig_based_active_reconstruction. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "ig_based_active_reconstruction/view_space.h"
#include "ig_based_active_reconstruction/view.h"

#include "ig_based_active_reconstruction_msgs/MovementCostMsg.h"


namespace ig_based_active_reconstruction
{

/*! abstract interface class that serves as an interface between the low level, robot specific planner and the view planner:
 * For each interface function apart from "initializePlanningFrame" there must exist an equivalent service call.
 */
class RobotPlanningInterface
{
public:
  class MovementCost;
  class PlanningSpaceInitializationInfo;
  class DataRetrievalModule;
  
  /*! Reception info
   */
  enum struct ReceptionInfo
  { 
    SUCCEEDED, 
    FAILED
  };
  
public:
  RobotPlanningInterface();
  
  /*! initializes the robot's planning space
   * @param _info information the robot needs to setup its planning space
   */
  virtual bool initializePlanningSpace( PlanningSpaceInitializationInfo& _info )=0;
  
  /*! returns the current view */
  virtual View getCurrentView()=0;
  
  /*! returns the view space that is available for planning (the idea is that it consists exclusively of poses that are considered to be reachable aforehand by the robot, considering its restraints
   * @param _space pointer to the ViewSpace object that should be filled
   * @return false if it failed
   */
  virtual bool getPlanningSpace( ViewSpace* _space )=0;
  
  /*! returns a sub space of the view space with all view points available within a certain range _distance of _view
   * @param _space pointer to the ViewSpace object that should be filled
   * @param _view view from which to take the distance
   * @return false if it failed
   */
  //virtual bool getSubPlanningSpace( ViewSpace* _space, View& _view, double _distance )=0;
  
  /*! executes the actions needed to retrieve new data, e.g. scanning movements until remode converges
   * @return information about what happened (data received, receival failed )
   */
  virtual ReceptionInfo retrieveData()=0;
  
  /*! returns the cost to move from the current view to the indicated view
   * @param _target_view the next view
   * @return cost to move to that view
   */
  virtual MovementCost movementCost( View& _target_view )=0;
  
  /*! returns the cost to move from start view to target view
   * @param _start_view the start view
   * @param _target_view the target view
   * @param _fill_additional_information if true then the different parts of the cost will be included in the additional fields as well
   * @return cost for the movement
   */
  virtual MovementCost movementCost( View& _start_view, View& _target_view, bool _fill_additional_information  )=0;
  
  /*! tells the robot to get the camera to a new view
   * @param _target_view where to move to
   * @return false if the operation failed
   */
  virtual bool moveTo( View& _target_view )=0;
  
private:
};

/*! class to hold the cost it takes to move along a path */
class RobotPlanningInterface::MovementCost
{
public:
  enum struct Exception
  {
    NONE, 
    COST_UNKNOWN, 
    INFINITE_COST, 
    INVALID_STATE, 
    INVALID_TARGET_STATE, 
    INVALID_START_STATE
  };
  
public:
  
  MovementCost():exception(Exception::NONE){};
  
  /*! converts the cost to a message
   */
  ig_based_active_reconstruction_msgs::MovementCostMsg toMsg();
  
  /*!
   * loads from msg
   */
  void fromMsg( ig_based_active_reconstruction_msgs::MovementCostMsg& _msg );
  
public:
  double cost; // keeping it simple
  //! possible exceptions:: INFINITE_COST: do not move to target view, INVALID_STATE: robot is in state which somehow prevents it from calculating a cost, but the movement might be possible
  Exception exception;
  std::vector<std::string> additional_field_names; //! names for additional information fields (optional)
  std::vector<double> additional_fields_values; //! values corresponding to the description in additional_fiel
};

class RobotPlanningInterface::PlanningSpaceInitializationInfo
{
public:
  class RobotSpaceInfo;
  
  boost::shared_ptr<RobotSpaceInfo> getSpecifics();
  
  /*!
   * template for setting the specifics, attempts to type cast
   */
  template<class T>
  void setSpecifics( boost::shared_ptr<T> _specifics );
protected:
  boost::shared_ptr<RobotSpaceInfo> robot_specific_info_;
};

class RobotPlanningInterface::PlanningSpaceInitializationInfo::RobotSpaceInfo
{
public:
  virtual std::string type()=0;
  
};

//! module for retrieving data, encapsulates the interface to the stereo reconstruction module (e.g. REMODE)
class RobotPlanningInterface::DataRetrievalModule
{
public:
  /*!
   * attempts to retrieve data and reports on success, call is blocking
   */
  virtual ReceptionInfo retrieveData()=0;
};

template<class T>
void RobotPlanningInterface::PlanningSpaceInitializationInfo::setSpecifics( boost::shared_ptr<T> _specifics )
{
  robot_specific_info_ = boost::dynamic_pointer_cast<RobotSpaceInfo>(_specifics);
}

}