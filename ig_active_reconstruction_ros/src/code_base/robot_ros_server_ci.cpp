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

#include <stdexcept>

#include "ig_active_reconstruction_ros/robot_ros_server_ci.hpp"
#include "ig_active_reconstruction_ros/robot_conversions.hpp"
#include "ig_active_reconstruction_ros/views_conversions.hpp"



namespace ig_active_reconstruction
{
  
namespace robot
{
  
  RosServerCI::RosServerCI( ros::NodeHandle nh, boost::shared_ptr<CommunicationInterface> linked_interface )
  : nh_(nh)
  , linked_interface_(linked_interface)
  {
    current_view_service_ = nh_.advertiseService("robot/current_view", &RosServerCI::currentViewService, this );
    data_service_ = nh_.advertiseService("robot/retrieve_data", &RosServerCI::retrieveDataService, this );
    cost_service_ = nh_.advertiseService("robot/movement_cost", &RosServerCI::movementCostService, this );
    robot_moving_service_ = nh_.advertiseService("robot/move_to", &RosServerCI::moveToService, this );
  }
  
  views::View RosServerCI::getCurrentView()
  {
    if( linked_interface_ == nullptr )
      throw std::runtime_error("robot::RosServerCI::Interface not linked.");
    
    return linked_interface_->getCurrentView();
  }
  
  RosServerCI::ReceptionInfo RosServerCI::retrieveData()
  {
    if( linked_interface_ == nullptr )
      throw std::runtime_error("robot::RosServerCI::Interface not linked.");
    
    return linked_interface_->retrieveData();
  }
  
  MovementCost RosServerCI::movementCost( views::View& target_view )
  {
    if( linked_interface_ == nullptr )
      throw std::runtime_error("robot::RosServerCI::Interface not linked.");
    
    return linked_interface_->movementCost( target_view );
  }
  
  MovementCost RosServerCI::movementCost( views::View& start_view, views::View& target_view, bool fill_additional_information  )
  {
    if( linked_interface_ == nullptr )
      throw std::runtime_error("robot::RosServerCI::Interface not linked.");
    
    return linked_interface_->movementCost( start_view, target_view, fill_additional_information  );
  }
  
  bool RosServerCI::moveTo( views::View& target_view )
  {
    if( linked_interface_ == nullptr )
      throw std::runtime_error("robot::RosServerCI::Interface not linked.");
    
    return linked_interface_->moveTo( target_view );
  }
  
  bool RosServerCI::currentViewService( ig_active_reconstruction_msgs::ViewRequest::Request& req, ig_active_reconstruction_msgs::ViewRequest::Response& res )
  {
    
    ROS_INFO("Received current view call.");
    if( linked_interface_ == nullptr )
    {
      return false;
    }
    
    views::View current_view = linked_interface_->getCurrentView();
    res.view = ros_conversions::viewToMsg(current_view);
    
    return true;
  }
  
  bool RosServerCI::retrieveDataService( ig_active_reconstruction_msgs::RetrieveData::Request& req, ig_active_reconstruction_msgs::RetrieveData::Response& res )
  {
    ROS_INFO("Received 'retrieve data' call.");
    if( linked_interface_ == nullptr )
    {
      ReceptionInfo inf = ReceptionInfo::FAILED;
      res.receive_info = ros_conversions::robotReceptionInfoToMsg(inf);
      return true;
    }
    
    ReceptionInfo rec_info = linked_interface_->retrieveData();
    
    res.receive_info = ros_conversions::robotReceptionInfoToMsg(rec_info);
    return true;
  }
  
  bool RosServerCI::movementCostService( ig_active_reconstruction_msgs::MovementCostCalculation::Request& req, ig_active_reconstruction_msgs::MovementCostCalculation::Response& res )
  {
    ROS_INFO("Received 'movement cost' call.");
    MovementCost cost;
    
    if( linked_interface_ == nullptr )
    {
      cost.exception = MovementCost::Exception::RECEPTION_FAILED;
    }
    else
    {
      views::View start_view = ros_conversions::viewFromMsg(req.start_view);
      views::View target_view = ros_conversions::viewFromMsg(req.target_view);
      bool fill_additional_info = req.additional_information;
      
      cost = linked_interface_->movementCost( start_view, target_view, fill_additional_info );
    }
    
    res.movement_cost = ros_conversions::movementCostToMsg(cost);
    return true;
  }
  
  bool RosServerCI::moveToService( ig_active_reconstruction_msgs::MoveToOrder::Request& req, ig_active_reconstruction_msgs::MoveToOrder::Response& res )
  {
    ROS_INFO("Received 'move to position' call.");
    if( linked_interface_ == nullptr )
    {
      res.success = false;
      return true;
    }
    
    views::View target_view  = ros_conversions::viewFromMsg(req.target_view);
    
    bool success = linked_interface_->moveTo(target_view);
    
    res.success = success;
    return true;
  }
}

}