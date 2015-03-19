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

#include "dense_reconstruction/youbot_planning.h"

namespace dense_reconstruction
{

class RemodeDataRetriever: public YoubotPlanner::DataRetrievalModule
{
public:
  /**
   * constructor expects a YoubotPlannerInstance pointer
   * @param _youbot_interface_namespace interface under which parameters will be sought
   */
  RemodeDataRetriever( YoubotPlanner* _robot_interface, std::string _youbot_interface_namespace="youbot_interface" );
    
  /**
   * returns a string that describes the movement executed, if any, e.g. 'InOutSpiral_0.05'
   * This used to name configuration files
   */
  virtual std::string movementConfigurationDescription();
  
  /**
   * attempts to retrieve data and reports on success, call is blocking
   */
  virtual RobotPlanningInterface::ReceiveInfo retrieveData();
  
  /**
   * returns true if a movement needs to be executed in order to retrieve data
   */
  virtual bool movementNeeded();
  
  /**
   * If a trajectory is necessary to retrieve data, this function is used to calculate it
   * @param _state state of the robot for which trajectory shall be calculated (currently only trajectories for arm links 2..4 are supported)
   * @param _retrieval_movement The trajectory
   * @param _additional_info at what time the trajectory shall start and at what it shall end
   * @return true if a trajectory is needed, false if not (which leaves the _retrieval_movement untouched)
   */
  virtual bool getRetrievalMovement( robot_state::RobotState& _state, movements::KinematicMovementDescription* _retrieval_movement, movements::KinematicMovementDescription::PathInfo* _additional_info );
  
  void remodeCallback( const sensor_msgs::PointCloud2ConstPtr& _msg );
private:
  YoubotPlanner* robot_interface_;
  double scanning_radius_; /// max radius of the scanning spirals default = 0.05
  
  ros::Subscriber remode_topic_subsriber_; /// to check whether remode has published
  ros::Publisher remode_commander_; /// interface to send commands to Remode
  
  bool remode_has_published_; /// set by callback function if remode data was published
  ros::Duration max_remode_wait_time_; ///max time to wait for remode to publish before receiving data assumes that it failed
  
  static std_msgs::String START_RECONSTRUCTION; /// string message commands for remode
  static std_msgs::String STOP_AND_SMOOTH; /// string message commands for remode
};

}