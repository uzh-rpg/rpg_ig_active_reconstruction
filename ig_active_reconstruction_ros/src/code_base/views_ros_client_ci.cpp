/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of ig_active_reconstruction, a ROS package for...well,

ig_active_reconstruction is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
ig_active_reconstruction is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with ig_active_reconstruction. If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdexcept>

#include "ig_active_reconstruction_ros/views_ros_client_ci.hpp"
#include "ig_active_reconstruction_ros/conversions.hpp"

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
  
  RosClientCI::ViewSpaceStatus RosClientCI::getPlanningSpace( ViewSpace* space )
  {
    ig_active_reconstruction_msgs::ViewSpaceRequest call;
    
    bool response = planning_space_receiver_.call(call);
    
    if( !response )
      return ViewSpaceStatus::BAD;
    
    *space = ros_conversions::viewSpaceFromMsg(call.response.viewspace);
    
    return ros_conversions::viewSpaceStatusFromMsg(call.response.viewspace_status);
  }
  
  void RosClientCI::getViewSpacePtr(ViewSpace* viewspace, ViewSpaceStatus& status)
  {
    status = ViewSpaceStatus::NONE_AVAILABLE;
    
    return;
  }
  
  RosClientCI::ViewSpaceUpdateResult RosClientCI::addViews( std::vector<View>& new_views )
  {
    ig_active_reconstruction_msgs::ViewSpaceUpdate call;
    
    for(View& view: new_views)
    {
      call.request.views.push_back( ros_conversions::viewToMsg(view) );
    }
    
    bool response = views_adder_.call(call);
    
    if( !response )
      return ViewSpaceUpdateResult::FAILED;
    
    return ros_conversions::viewSpaceUpdateResultFromMsg(call.response.update_result);
  }
  
  RosClientCI::ViewSpaceUpdateResult RosClientCI::addView( View new_view )
  {
    ig_active_reconstruction_msgs::ViewSpaceUpdate call;
    
    call.request.views.push_back( ros_conversions::viewToMsg(new_view) );
    
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
    
    bool response = views_deleter_.call(call);
    if( !response )
      return ViewSpaceUpdateResult::FAILED;
    
    return ros_conversions::viewSpaceUpdateResultFromMsg(call.response.update_result);
  }
  
  RosClientCI::ViewSpaceUpdateResult RosClientCI::deleteView( View::IdType view_id )
  {
    ig_active_reconstruction_msgs::DeleteViews call;
    
    call.request.ids.push_back(view_id);
    
    bool response = views_deleter_.call(call);
    if( !response )
      return ViewSpaceUpdateResult::FAILED;
    
    return ros_conversions::viewSpaceUpdateResultFromMsg(call.response.update_result);
  }
  
  
}

}