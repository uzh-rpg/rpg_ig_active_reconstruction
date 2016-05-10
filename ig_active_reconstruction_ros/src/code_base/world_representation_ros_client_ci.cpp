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

#include "ig_active_reconstruction_ros/world_representation_ros_client_ci.hpp"
#include "ig_active_reconstruction_ros/world_conversions.hpp"

#include "ig_active_reconstruction_msgs/InformationGainCalculation.h"
#include "ig_active_reconstruction_msgs/MapMetricCalculation.h"
#include "ig_active_reconstruction_msgs/StringList.h"


namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
  RosClientCI::RosClientCI( ros::NodeHandle nh )
  : nh_(nh)
  {
    view_ig_computation_ = nh.serviceClient<ig_active_reconstruction_msgs::InformationGainCalculation>("world/information_gain");
    map_metric_computation_ = nh.serviceClient<ig_active_reconstruction_msgs::MapMetricCalculation>("world/map_metric");
    available_ig_receiver_ = nh.serviceClient<ig_active_reconstruction_msgs::StringList>("world/ig_list");
    available_mm_receiver_ = nh.serviceClient<ig_active_reconstruction_msgs::StringList>("world/mm_list");
  }
  
  RosClientCI::ResultInformation RosClientCI::computeViewIg(IgRetrievalCommand& command, ViewIgResult& output_ig)
  {
    ig_active_reconstruction_msgs::InformationGainCalculation call;
    call.request.command = ros_conversions::igRetrievalCommandToMsg(command);
    
    ROS_INFO("Demanding information gain.");
    bool response = view_ig_computation_.call(call);
    
    if(!response)
    {
      unsigned int number_of_metrics = (!command.metric_ids.empty())?command.metric_ids.size():command.metric_names.size();
      IgRetrievalResult failed;
      failed.status = ResultInformation::FAILED;
      failed.predicted_gain = 0;
      
      for(unsigned int i=0; i<number_of_metrics; ++i )
      {
	output_ig.push_back(failed);
      }
      return ResultInformation::FAILED;
    }
    else
    {
      for(ig_active_reconstruction_msgs::InformationGain& ig: call.response.expected_information)
      {
	IgRetrievalResult result = ros_conversions::igRetrievalResultFromMsg(ig);
	output_ig.push_back(result);
      }
      return ResultInformation::SUCCEEDED;
    }
  }
  
  RosClientCI::ResultInformation RosClientCI::computeMapMetric(MapMetricRetrievalCommand& command, MapMetricRetrievalResultSet& output)
  {
    ig_active_reconstruction_msgs::MapMetricCalculation call;
    
    for(std::string& name: command.metric_names)
    {
      call.request.metric_names.push_back(name);
    }
    
    ROS_INFO("Demanding map metric.");
    bool response = map_metric_computation_.call(call);
    
    if(!response)
    {
      MapMetricRetrievalResult failed;
      failed.status = ResultInformation::FAILED;
      failed.value = 0;
      for(unsigned int i=0; i<command.metric_names.size(); ++i)
      {
	output.push_back(failed);
      }
      return ResultInformation::FAILED;
    }
    else
    {
      for(ig_active_reconstruction_msgs::InformationGain& map_metric: call.response.results)
      {
	MapMetricRetrievalResult result;
	result.status = ros_conversions::resultInformationFromMsg(map_metric.status);
	result.value = map_metric.predicted_gain;
	output.push_back(result);
      }
      return ResultInformation::SUCCEEDED;
    }
  }
  
  void RosClientCI::availableIgMetrics( std::vector<MetricInfo>& available_ig_metrics )
  {
    
    ROS_INFO("Demanding available information gains.");
    ig_active_reconstruction_msgs::StringList call;
    bool response = available_ig_receiver_.call(call);
    
    if(!response)
      return;
    
    for(unsigned int i=0; i<call.response.names.size(); ++i)
    {
      MetricInfo new_metric;
      new_metric.name = call.response.names[i];
      new_metric.id = call.response.ids[i];
      available_ig_metrics.push_back(new_metric);
    }
  }
  
  void RosClientCI::availableMapMetrics( std::vector<MetricInfo>& available_map_metrics )
  {
    
    ROS_INFO("Demanding available map metrics.");
    ig_active_reconstruction_msgs::StringList call;
    bool response = available_mm_receiver_.call(call);
    
    if(!response)
      return;
    
    for(unsigned int i=0; i<call.response.names.size(); ++i)
    {
      MetricInfo new_metric;
      new_metric.name = call.response.names[i];
      new_metric.id = call.response.ids[i];
      available_map_metrics.push_back(new_metric);
    }
  }
  
}

}