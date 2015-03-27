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
#include "octomap_msgs/Octomap.h"

namespace dense_reconstruction
{

class StereoCameraDataRetriever: public YoubotPlanner::DataRetrievalModule
{
public:
  /**
   * constructor expects a YoubotPlannerInstance pointer
   * @param _youbot_interface_namespace interface under which parameters will be sought
   */
  StereoCameraDataRetriever( YoubotPlanner* _robot_interface, std::string _youbot_interface_namespace="youbot_interface" );
    
  /**
   * returns a string that describes the movement executed, if any, e.g. 'InOutSpiral_0.05' or 'Static' (for no movement at all)
   * This used to name configuration files
   */
  virtual std::string movementConfigurationDescription();
  
  /**
   * attempts to retrieve data and reports on success, call is blocking
   * success is reported when octomap has processed the data and published itself.
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
  
  /** retrieves a pointcloud to republish it to octomap if asked
   */
  void pclCallback( const sensor_msgs::PointCloud2ConstPtr& _msg );
  
  void octomapCallback( const octomap_msgs::OctomapConstPtr& _msg );
private:
  YoubotPlanner* robot_interface_;
  
  bool octomap_has_published_;
  bool republish_;
  
  ros::Subscriber octomap_topic_subscriber_; /// to check whether octomap has published
  ros::Subscriber pcl_subscriber_;
  ros::Publisher pcl_publisher_;
  ros::Duration max_pointcloud_wait_time_; /// [s]max time to wait for pcl to come in and again to see if octomap has published
  ros::Duration max_octomap_wait_time_; /// [s]max time to wait for octomap to publish
  
};

}