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

#include "ig_active_reconstruction_ros/world_conversions.hpp"
#include "movements/ros_movements.h"
#include <stdexcept>
#include <boost/foreach.hpp>

namespace ig_active_reconstruction
{
    
namespace ros_conversions
{
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
    
    BOOST_FOREACH( geometry_msgs::Pose& pose, command_msg.poses )
    {
      command.path.push_back( movements::fromROS(pose) );
    }
    
    BOOST_FOREACH( std::string& name, command_msg.metric_names )
    {
      command.metric_names.push_back(name);
    }
    
    BOOST_FOREACH( unsigned int& id, command_msg.metric_ids )
    {
      command.metric_ids.push_back(id);
    }
    
    command.config = igRetrievalConfigFromMsg(command_msg.config);
    
    return command;
  }
  
  ig_active_reconstruction_msgs::InformationGainRetrievalCommand igRetrievalCommandToMsg(world_representation::CommunicationInterface::IgRetrievalCommand& command)
  {
    ig_active_reconstruction_msgs::InformationGainRetrievalCommand command_msg;
    
    BOOST_FOREACH(movements::Pose& pose, command.path)
    {
      command_msg.poses.push_back( movements::toROS(pose) );
    }
    
    BOOST_FOREACH(std::string& name, command.metric_names)
    {
      command_msg.metric_names.push_back(name);
    }
    
    BOOST_FOREACH(unsigned int& id, command.metric_ids)
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