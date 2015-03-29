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
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <tf/transform_listener.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <movements/core>
#include <movements/ros_movements.h>
#include <movements/translation.h>
#include <movements/linear_movement.h>
#include <movements/in_out_spiral.h>

#include "dense_reconstruction/youbot_planning.h"
#include "dense_reconstruction/robot_planning_interface.h"

namespace dense_reconstruction
{
  
/// class that autonomously extracts hand-eye pose correspondences in a robotic setup and estimates the hand-eye-calibration from it
class FlyingStereoCameraInterface: public RobotPlanningInterface
{
public:
  
  /// constructor
  /** initializes the youbot planner, moves the robot to the initial pose
   * @param _n handle of the node the calibrator runs in
   * @throws ROS_FATAL if not all necessary parameters are given and shuts down the node
   */
  FlyingStereoCameraInterface( ros::NodeHandle* _n );
  
  /// destructor
  ~FlyingStereoCameraInterface();
  
  /** returns the name of the global planning frame (currently "dr_origin" for 'dense reconstruction origin) and does all calculations needed in order to set up the tf tree for that frame, e.g. initialize SVO, save transformation from SVO frame (world) to (dr_origin) etc.
   */
  virtual std::string initializePlanningFrame();
    
  /** initializes the robot's planning space
   * @param _info information the robot needs to setup its planning space
   */
  virtual bool initializePlanningSpace( PlanningSpaceInitializationInfo& _info );
  
  /** returns the current view */
  virtual View getCurrentView();
  
  /** returns the view space that is available for planning (the idea is that it consists exclusively of poses that are considered to be reachable aforehand by the robot, considering its restraints
   * @param _space pointer to the ViewSpace object that should be filled
   * @return false if it failed
   */
  virtual bool getPlanningSpace( ViewSpace* _space );
  
  /** returns a sub space of the view space with all view points available within a certain range _distance of _view
   * @param _space pointer to the ViewSpace object that should be filled
   * @param _view view from which to take the distance
   * @return false if it failed
   */
  //virtual bool getSubPlanningSpace( ViewSpace* _space, View& _view, double _distance );
  
  /** executes the actions needed to retrieve new data, e.g. scanning movements until remode converges
   * @return information about what happened (data received, receival failed )
   */
  virtual RobotPlanningInterface::ReceiveInfo retrieveData();
  
  /** returns the cost to move from the current view to the indicated view
   * @param _target_view the next view
   * @throws std::runtime_error if the type of the _target_view's associated info can't be casted to the Youbot type
   * @return cost to move to that view
   */
  virtual RobotPlanningInterface::MovementCost movementCost( View& _target_view );
  
  /** returns the cost to move from start view to target view
   * @param _start_view the start view
   * @param _target_view the target view
   * @param _fill_additional_information if true then the different parts of the cost will be included in the additional fields as well
   * @return cost for the movement
   */
  virtual MovementCost movementCost( View& _start_view, View& _target_view, bool _fill_additional_information=false );
  
   /** tells the robot to get the camera to a new view
   * @param _target_view where to move to
   * @return false if the operation failed
   */
  virtual bool moveTo( View& _target_view );
  
  /** service to initialize the planning space
   */
  bool planningSpaceInitService( dense_reconstruction::PlanningSpaceInitializationInfoMsg::Request& _req, dense_reconstruction::PlanningSpaceInitializationInfoMsg::Response& _res );
  
  /** service to return the feasible view space
   */
  bool feasibleViewSpaceRequestService( dense_reconstruction::FeasibleViewSpaceRequest::Request& _req, dense_reconstruction::FeasibleViewSpaceRequest::Response& _res );
  
  /** service to obtain the current view
   */
  bool currentViewService( dense_reconstruction::ViewRequest::Request& _req, dense_reconstruction::ViewRequest::Response& _res );
  
  /** service that tells the youbot to retrieve data
   */
  bool retrieveDataService( dense_reconstruction::RetrieveData::Request& _req, dense_reconstruction::RetrieveData::Response& _res );
  
  /** service for movement costs (see equivalent function for further explanations )
   */
  bool movementCostService( dense_reconstruction::MovementCostCalculation::Request& _req, dense_reconstruction::MovementCostCalculation::Response& _res );
  
  /** service to tell the youbot which view pose to assume
   */
  bool moveToService( dense_reconstruction::MoveToOrder::Request& _req, dense_reconstruction::MoveToOrder::Response& _res );
  
  /** service that when called finalizes the initialization - it sets up the tf frames,
   * that is e.g. the transformation from cam_pos to the robot and to dr_origin
   */
  bool setupTFService( std_srvs::Empty::Request& _req, std_srvs::Empty::Response& _res );
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  
  ros::NodeHandle* ros_node_;
  ros::ServiceClient eye_client_;
  ros::ServiceClient hand_client_;
  ros::ServiceServer planning_space_initialization_server_;
  ros::ServiceServer feasible_view_space_request_server_;
  ros::ServiceServer current_view_server_;
  ros::ServiceServer retrieve_data_server_;
  ros::ServiceServer movement_cost_server_;
  ros::ServiceServer move_to_server_;
  ros::ServiceServer setup_tf_server_;
  
  
  boost::shared_ptr<YoubotPlanner::DataRetrievalModule> data_retreiver_;
  
  std::string view_planning_frame_; /// planning frame for views
  std::string end_effector_planning_frame_; /// name of the frame for which the end effector pose shall be controlled
  
  unsigned int current_view_;
  ViewSpace view_space_;
  
  
  tf::TransformListener tf_listener_;
  
  
};

}