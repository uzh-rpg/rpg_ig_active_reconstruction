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

#include "ig_active_reconstruction_ros/views_ros_client_ci.hpp"
#include "ig_active_reconstruction_ros/views_conversions.hpp"

#include "ig_active_reconstruction_msgs/DeleteViews.h"
#include "ig_active_reconstruction_msgs/ViewSpaceRequest.h"
#include "ig_active_reconstruction_msgs/ViewSpaceUpdate.h"


namespace ig_active_reconstruction
{
  
namespace views
{
  
  RosClientCI::RosClientCI( ros::NodeHandle nh )
  : nh_(nh)
  {
    planning_space_receiver_ = nh.serviceClient<ig_active_reconstruction_msgs::ViewSpaceRequest>("views/space");
    views_adder_ = nh.serviceClient<ig_active_reconstruction_msgs::ViewSpaceUpdate>("views/add");
    views_deleter_ = nh.serviceClient<ig_active_reconstruction_msgs::DeleteViews>("views/delete");
  }
  
  const ViewSpace& RosClientCI::getViewSpace()
  {
    ig_active_reconstruction_msgs::ViewSpaceRequest call;
    
    ROS_INFO("Demanding viewspace.");
    bool response = planning_space_receiver_.call(call);
    
    if( response )
      viewspace_ = ros_conversions::viewSpaceFromMsg(call.response.viewspace);
    
    return viewspace_;
  }
  
  RosClientCI::ViewSpaceUpdateResult RosClientCI::addViews( std::vector<View>& new_views )
  {
    ig_active_reconstruction_msgs::ViewSpaceUpdate call;
    
    for(View& view: new_views)
    {
      call.request.views.push_back( ros_conversions::viewToMsg(view) );
    }
    
    
    ROS_INFO("Demanding to add view(s).");
    bool response = views_adder_.call(call);
    
    if( !response )
      return ViewSpaceUpdateResult::FAILED;
    
    return ros_conversions::viewSpaceUpdateResultFromMsg(call.response.update_result);
  }
  
  RosClientCI::ViewSpaceUpdateResult RosClientCI::addView( View new_view )
  {
    ig_active_reconstruction_msgs::ViewSpaceUpdate call;
    
    call.request.views.push_back( ros_conversions::viewToMsg(new_view) );
    
    
    ROS_INFO("Demanding to add view.");
    bool response = views_adder_.call(call);
    
    if( !response )
      return ViewSpaceUpdateResult::FAILED;
    
    return ros_conversions::viewSpaceUpdateResultFromMsg(call.response.update_result);
  }
  
  RosClientCI::ViewSpaceUpdateResult RosClientCI::deleteViews( std::vector<View::IdType>& view_ids )
  {
    ig_active_reconstruction_msgs::DeleteViews call;
    
    for(View::IdType& id:view_ids)
    {
      call.request.ids.push_back(id);
    }
    
    
    ROS_INFO("Demanding to delete view(s).");
    bool response = views_deleter_.call(call);
    if( !response )
      return ViewSpaceUpdateResult::FAILED;
    
    return ros_conversions::viewSpaceUpdateResultFromMsg(call.response.update_result);
  }
  
  RosClientCI::ViewSpaceUpdateResult RosClientCI::deleteView( View::IdType view_id )
  {
    ig_active_reconstruction_msgs::DeleteViews call;
    
    call.request.ids.push_back(view_id);
    
    
    ROS_INFO("Demanding to delete view.");
    bool response = views_deleter_.call(call);
    if( !response )
      return ViewSpaceUpdateResult::FAILED;
    
    return ros_conversions::viewSpaceUpdateResultFromMsg(call.response.update_result);
  }
  
  
}

}