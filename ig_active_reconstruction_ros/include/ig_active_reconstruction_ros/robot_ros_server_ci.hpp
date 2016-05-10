/* Copyright (c) 2016, Stefan Isler, islerstefan@bluewin.ch
 * (ETH Zurich / Robotics and Perception Group, University of Zurich, Switzerland)
 *
 * This file is part of ig_active_reconstruction, software for information gain based, active reconstruction.
 *
 * ig_active_reconstruction is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * ig_active_reconstruction is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * Please refer to the GNU Lesser General Public License for details on the license,
 * on <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "ros/ros.h"
#include "ig_active_reconstruction/robot_communication_interface.hpp"

#include "ig_active_reconstruction_msgs/ViewRequest.h"
#include "ig_active_reconstruction_msgs/RetrieveData.h"
#include "ig_active_reconstruction_msgs/MovementCostCalculation.h"
#include "ig_active_reconstruction_msgs/MoveToOrder.h"

namespace ig_active_reconstruction
{
  
namespace robot
{
  
  /*! Generic "resident" ROS robot communication interface implementation.
   * 
   * Uses the ROS communication interface (topics, services etc.) to receive requests and feed it into
   * a robot::CommunicationInterface implementation.
   */
  class RosServerCI: public CommunicationInterface
  {
  public:
    /*! Constructor
     * @param nh ROS node handle defines the namespace in which ROS communication will be carried out.
     * @param linked_interface (optional) directly add the interface that is linked internally (to which requests are forwarded.
     */
    RosServerCI( ros::NodeHandle nh, boost::shared_ptr<CommunicationInterface> linked_interface = nullptr );
    
    /*! Set a new linked interface to which the class forwards all requests.
     * @param linked_interface Interface pointer.
     */
    void setLinkedInterface( boost::shared_ptr<CommunicationInterface> linked_interface );
  
    /*! returns the current view */
    virtual views::View getCurrentView();
    
    /*! Commands robot to retrieve new data.
     * 
     * @throws std::runtime_error If receiving data failed.
    * @return information about what happened (data received, receival failed )
    */
    virtual ReceptionInfo retrieveData();
    
    /*! Returns the cost to move from the current view to the indicated view
     * 
     * @throws std::runtime_error If receiving data failed.
    * @param target_view the next view
    * @return cost to move to that view
    */
    virtual MovementCost movementCost( views::View& target_view );
    
    /*! returns the cost to move from start view to target view
    * @param start_view the start view
    * @param target_view the target view
    * @param fill_additional_information if true then the different parts of the cost will be included in the additional fields as well
    * @return cost for the movement
    */
    virtual MovementCost movementCost( views::View& start_view, views::View& target_view, bool fill_additional_information  );
    
    /*! Tells the robot to get the camera to a new view
    * @param _target_view where to move to
    * @return false if the operation failed
    */
    virtual bool moveTo( views::View& target_view );
    
  protected:
    bool currentViewService( ig_active_reconstruction_msgs::ViewRequest::Request& req, ig_active_reconstruction_msgs::ViewRequest::Response& res );
    
    bool retrieveDataService( ig_active_reconstruction_msgs::RetrieveData::Request& req, ig_active_reconstruction_msgs::RetrieveData::Response& res );
    
    bool movementCostService( ig_active_reconstruction_msgs::MovementCostCalculation::Request& req, ig_active_reconstruction_msgs::MovementCostCalculation::Response& res );
    
    bool moveToService( ig_active_reconstruction_msgs::MoveToOrder::Request& req, ig_active_reconstruction_msgs::MoveToOrder::Response& res );
    
  protected:
    ros::NodeHandle nh_;
    
    boost::shared_ptr<CommunicationInterface> linked_interface_; //! Linked interface.
    
    ros::ServiceServer current_view_service_;
    ros::ServiceServer data_service_;
    ros::ServiceServer cost_service_;
    ros::ServiceServer robot_moving_service_;
  };
  
}

}