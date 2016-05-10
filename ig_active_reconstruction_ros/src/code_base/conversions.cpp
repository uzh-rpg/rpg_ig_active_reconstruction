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

#if 0

#include "ig_active_reconstruction_ros/conversions.hpp"
#include "movements/ros_movements.h"
#include <stdexcept>

namespace ig_active_reconstruction
{
    
namespace ros_conversions
{
  ig_active_reconstruction_msgs::MovementCostMsg movementCostToMsg(const robot::MovementCost& cost)
  {
    ig_active_reconstruction_msgs::MovementCostMsg msg;
    msg.cost = cost.cost;
    msg.exception = static_cast<uint32_t>(cost.exception);
    msg.additional_fields_names = cost.additional_field_names;
    msg.additional_fields_values = cost.additional_fields_values;
    return msg;
  }

  robot::MovementCost movementCostFromMsg( ig_active_reconstruction_msgs::MovementCostMsg& _msg )
  {
    robot::MovementCost cost;
    cost.cost = _msg.cost;
    cost.exception = robot::MovementCost::Exception(_msg.exception);
    cost.additional_field_names = _msg.additional_fields_names;
    cost.additional_fields_values = _msg.additional_fields_values;
    return cost;
  }
  
  ig_active_reconstruction_msgs::ViewMsg viewToMsg( views::View& view )
  {
    ig_active_reconstruction_msgs::ViewMsg msg;
    msg.pose = movements::toROS( view.pose() );
    msg.source_frame = view.sourceFrame();
    msg.is_bad = view.bad();
    msg.visited = view.timesVisited();
    msg.is_reachable = view.reachable();
    msg.associated_names = view.additionalFieldsNames();
    msg.associated_values = view.additionalFieldsValues();
    msg.index = view.index();
    return msg;
  }
  
  views::View viewFromMsg( ig_active_reconstruction_msgs::ViewMsg& msg )
  {
    views::View view(msg.index);
    view.pose() = movements::fromROS(msg.pose);
    view.sourceFrame() = msg.source_frame;
    view.reachable() = msg.is_reachable;
    view.bad() = msg.is_bad;
    view.timesVisited() = msg.visited;
    view.additionalFieldsNames() = msg.associated_names;
    view.additionalFieldsValues() = msg.associated_values;
    
    return view;
  }
  
  ig_active_reconstruction_msgs::ViewSpaceMsg viewSpaceToMsg( views::ViewSpace& view_space )
  {
    ig_active_reconstruction_msgs::ViewSpaceMsg msg;
    for( views::View& view: view_space )
    {
      msg.views.push_back( viewToMsg(view) );
    }
  }
  
  views::ViewSpace viewSpaceFromMsg( ig_active_reconstruction_msgs::ViewSpaceMsg& msg )
  {
    views::ViewSpace view_space;
    
    for( auto& view: msg.views )
    {
      view_space.push_back( viewFromMsg(view) );
    }
  }
  
  robot::CommunicationInterface::ReceptionInfo robotReceptionInfoFromMsg( int& receive_info )
  {
    if( receive_info==0 )
      return robot::CommunicationInterface::ReceptionInfo::SUCCEEDED;
    else if( receive_info==1 )
      return robot::CommunicationInterface::ReceptionInfo::FAILED;
  }
  
  int robotReceptionInfoToMsg( robot::CommunicationInterface::ReceptionInfo& info )
  {
    if( info==robot::CommunicationInterface::ReceptionInfo::SUCCEEDED )
      return 0;
    else
      return 1;
  }
  
  views::CommunicationInterface::ViewSpaceStatus viewSpaceStatusFromMsg( int& msg )
  {
    switch(msg)
    {
      case 0: return views::CommunicationInterface::ViewSpaceStatus::OK;
      case 1: return views::CommunicationInterface::ViewSpaceStatus::BAD;
      case 2: return views::CommunicationInterface::ViewSpaceStatus::NONE_AVAILABLE;
      default: throw std::invalid_argument("ig_active_reconstruction::ros_conversions::viewSpaceStatusFromMsg:: Invalid msg received.");
    };
  }
  
  int viewSpaceStatusToMsg( views::CommunicationInterface::ViewSpaceStatus& status )
  {
    switch(status)
    {
      case views::CommunicationInterface::ViewSpaceStatus::OK: return 0;
      case views::CommunicationInterface::ViewSpaceStatus::BAD: return 1;
      case views::CommunicationInterface::ViewSpaceStatus::NONE_AVAILABLE: return 2;
      default: throw std::invalid_argument("ig_active_reconstruction::ros_conversions::viewSpaceStatusToMsg:: Invalid status received.");
    };
  }
  
  views::CommunicationInterface::ViewSpaceUpdateResult viewSpaceUpdateResultFromMsg(int& msg)
  {
    switch(msg)
    {
      case 0: return views::CommunicationInterface::ViewSpaceUpdateResult::SUCCEEDED;
      case 1: return views::CommunicationInterface::ViewSpaceUpdateResult::FAILED;
      case 2: return views::CommunicationInterface::ViewSpaceUpdateResult::NOT_AVAILABLE;
    };
  }
  
  int viewSpaceUpdateResultToMsg(views::CommunicationInterface::ViewSpaceUpdateResult& res)
  {
    switch(res)
    {
      case views::CommunicationInterface::ViewSpaceUpdateResult::SUCCEEDED: return 0;
      case views::CommunicationInterface::ViewSpaceUpdateResult::FAILED: return 1;
      case views::CommunicationInterface::ViewSpaceUpdateResult::NOT_AVAILABLE: return 2;
    };
  }
  
  world_representation::CommunicationInterface::IgRetrievalConfig igRetrievalConfigFromMsg(ig_active_reconstruction_msgs::InformationGainRetrievalConfig& config_msg)
  {
    world_representation::CommunicationInterface::IgRetrievalConfig config;
    config.ray_resolution_x = config_msg.ray_resolution_x;
    config.ray_resolution_y = config_msg.ray_resolution_y;
    config.ray_window.min_x_perc = config_msg.ray_window.min_x_perc;
    config.ray_window.max_x_perc = config_msg.ray_window.max_x_perc;
    config.ray_window.min_y_perc = config_msg.ray_window.min_y_perc;
    config.ray_window.max_y_perc = config_msg.ray_window.max_y_perc;
    config.max_ray_depth = config_msg.max_ray_depth;
    
    return config;
  }
  
  ig_active_reconstruction_msgs::InformationGainRetrievalConfig igRetrievalConfigToMsg(world_representation::CommunicationInterface::IgRetrievalConfig& config)
  {
    ig_active_reconstruction_msgs::InformationGainRetrievalConfig config_msg;
    config_msg.ray_resolution_x = config.ray_resolution_x;
    config_msg.ray_resolution_y = config.ray_resolution_y;
    config_msg.ray_window.min_x_perc = config.ray_window.min_x_perc;
    config_msg.ray_window.max_x_perc = config.ray_window.max_x_perc;
    config_msg.ray_window.min_y_perc = config.ray_window.min_y_perc;
    config_msg.ray_window.max_y_perc = config.ray_window.max_y_perc;
    config_msg.max_ray_depth = config.max_ray_depth;
    return config_msg;
  }
  
  world_representation::CommunicationInterface::IgRetrievalCommand igRetrievalCommandFromMsg(ig_active_reconstruction_msgs::InformationGainRetrievalCommand& command_msg)
  {
    world_representation::CommunicationInterface::IgRetrievalCommand command;
    
    for( geometry_msgs::Pose& pose: command_msg.poses )
    {
      command.path.push_back( movements::fromROS(pose) );
    }
    
    for( std::string& name: command_msg.metric_names )
    {
      command.metric_names.push_back(name);
    }
    
    for( unsigned int& id: command_msg.metric_ids )
    {
      command.metric_ids.push_back(id);
    }
    
    command.config = igRetrievalConfigFromMsg(command_msg.config);
    
    return command;
  }
  
  ig_active_reconstruction_msgs::InformationGainRetrievalCommand igRetrievalCommandToMsg(world_representation::CommunicationInterface::IgRetrievalCommand& command)
  {
    ig_active_reconstruction_msgs::InformationGainRetrievalCommand command_msg;
    
    for(movements::Pose& pose: command.path)
    {
      command_msg.poses.push_back( movements::toROS(pose) );
    }
    
    for(std::string& name: command.metric_names)
    {
      command_msg.metric_names.push_back(name);
    }
    
    for(unsigned int& id: command.metric_ids)
    {
      command_msg.metric_ids.push_back(id);
    }
    
    command_msg.config = igRetrievalConfigToMsg(command.config);
    
    return command_msg;
  }
  
  world_representation::CommunicationInterface::ResultInformation resultInformationFromMsg(int& msg)
  {
    switch(msg)
    {
      case 0: return world_representation::CommunicationInterface::ResultInformation::SUCCEEDED;
      case 1: return world_representation::CommunicationInterface::ResultInformation::FAILED;
      case 2: return world_representation::CommunicationInterface::ResultInformation::UNKNOWN_METRIC;
      default: throw std::invalid_argument("ig_active_reconstruction::ros_conversions::resultInformationFromMsg:: Invalid status received.");
    };
  }
  
  int resultInformationToMsg(world_representation::CommunicationInterface::ResultInformation& msg)
  {
    switch(msg)
    {
      case world_representation::CommunicationInterface::ResultInformation::SUCCEEDED: return 0;
      case world_representation::CommunicationInterface::ResultInformation::FAILED: return 1;
      case world_representation::CommunicationInterface::ResultInformation::UNKNOWN_METRIC: return 2;
      default: throw std::invalid_argument("ig_active_reconstruction::ros_conversions::resultInformationToMsg:: Invalid status received.");
    };
  }
  
  world_representation::CommunicationInterface::IgRetrievalResult igRetrievalResultFromMsg(ig_active_reconstruction_msgs::InformationGain& msg)
  {
    world_representation::CommunicationInterface::IgRetrievalResult result;
    
    result.status = resultInformationFromMsg(msg.status);
    result.predicted_gain = msg.predicted_gain;
    
    return result;
  }
  
  ig_active_reconstruction_msgs::InformationGain igRetrievalResultToMsg(world_representation::CommunicationInterface::IgRetrievalResult& result)
  {
    ig_active_reconstruction_msgs::InformationGain msg;
    
    msg.status = resultInformationToMsg(result.status);
    msg.predicted_gain = result.predicted_gain;
    
    return msg;
  }
}

}

#endif