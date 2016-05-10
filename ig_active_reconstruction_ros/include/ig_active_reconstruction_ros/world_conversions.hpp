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

#include "ig_active_reconstruction_msgs/InformationGainRetrievalCommand.h"
#include "ig_active_reconstruction_msgs/InformationGain.h"

#include "ig_active_reconstruction/world_representation_communication_interface.hpp"

namespace ig_active_reconstruction
{
    
namespace ros_conversions
{
    world_representation::CommunicationInterface::IgRetrievalConfig igRetrievalConfigFromMsg(ig_active_reconstruction_msgs::InformationGainRetrievalConfig& config);
    
    ig_active_reconstruction_msgs::InformationGainRetrievalConfig igRetrievalConfigToMsg(world_representation::CommunicationInterface::IgRetrievalConfig& config);
    
    world_representation::CommunicationInterface::IgRetrievalCommand igRetrievalCommandFromMsg(ig_active_reconstruction_msgs::InformationGainRetrievalCommand& command_msg);
    
    ig_active_reconstruction_msgs::InformationGainRetrievalCommand igRetrievalCommandToMsg(world_representation::CommunicationInterface::IgRetrievalCommand& command);
    
    world_representation::CommunicationInterface::ResultInformation resultInformationFromMsg(int& msg);
    
    int resultInformationToMsg(world_representation::CommunicationInterface::ResultInformation& msg);
    
    world_representation::CommunicationInterface::IgRetrievalResult igRetrievalResultFromMsg(ig_active_reconstruction_msgs::InformationGain& msg);
    
    ig_active_reconstruction_msgs::InformationGain igRetrievalResultToMsg(world_representation::CommunicationInterface::IgRetrievalResult& msg);
}

}