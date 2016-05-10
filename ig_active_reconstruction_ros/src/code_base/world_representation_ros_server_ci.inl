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

#define TEMPT template<template<typename> class POINTER_TYPE>
#define CSCOPE RosServerCI<POINTER_TYPE>

#include <stdexcept>
#include <boost/foreach.hpp>

//#include "ig_active_reconstruction_ros/world_representation_ros_server_ci.hpp"
#include "ig_active_reconstruction_ros/world_conversions.hpp"


namespace ig_active_reconstruction
{
  
namespace world_representation
{
  TEMPT
  CSCOPE::RosServerCI( ros::NodeHandle nh, POINTER_TYPE<CommunicationInterface> linked_interface )
  : nh_(nh)
  , linked_interface_(linked_interface)
  {
    view_ig_computation_ = nh.advertiseService("world/information_gain", &CSCOPE::igComputationService, this );
    map_metric_computation_ = nh.advertiseService("world/map_metric", &CSCOPE::mmComputationService, this );
    available_ig_receiver_ = nh.advertiseService("world/ig_list", &CSCOPE::availableIgService, this );
    available_mm_receiver_ = nh.advertiseService("world/mm_list", &CSCOPE::availableMmService, this );
  }
  
  TEMPT
  typename CSCOPE::ResultInformation CSCOPE::computeViewIg(IgRetrievalCommand& command, ViewIgResult& output_ig)
  {
    if( linked_interface_ == NULL )
      throw std::runtime_error("world_representation::CSCOPE::Interface not linked.");
    
    return linked_interface_->computeViewIg(command, output_ig);
  }
  
  TEMPT
  typename CSCOPE::ResultInformation CSCOPE::computeMapMetric(MapMetricRetrievalCommand& command, MapMetricRetrievalResultSet& output)
  {
    if( linked_interface_ == NULL )
      throw std::runtime_error("world_representation::CSCOPE::Interface not linked.");
    
    return linked_interface_->computeMapMetric(command, output);
  }
  
  TEMPT
  void CSCOPE::availableIgMetrics( std::vector<MetricInfo>& available_ig_metrics )
  {
    if( linked_interface_ == NULL )
      throw std::runtime_error("world_representation::CSCOPE::Interface not linked.");
    
    return linked_interface_->availableIgMetrics( available_ig_metrics );
  }
  
  TEMPT
  void CSCOPE::availableMapMetrics( std::vector<MetricInfo>& available_map_metrics )
  {
    if( linked_interface_ == NULL )
      throw std::runtime_error("world_representation::CSCOPE::Interface not linked.");
    
    return linked_interface_->availableMapMetrics( available_map_metrics );
  }
  
  TEMPT
  bool CSCOPE::igComputationService( ig_active_reconstruction_msgs::InformationGainCalculation::Request& req, ig_active_reconstruction_msgs::InformationGainCalculation::Response& res )
  {
    ROS_INFO("Received 'ig computation' call.");
    if( linked_interface_ == NULL )
    {
      ig_active_reconstruction_msgs::InformationGain failed;
      failed.predicted_gain = 0;
      ResultInformation failed_status = ResultInformation::FAILED;
      failed.status = ros_conversions::resultInformationToMsg(failed_status);
      unsigned int number_of_metrics = (!req.command.metric_ids.empty())?req.command.metric_ids.size():req.command.metric_names.size();
      for(unsigned int i=0; i<number_of_metrics; ++i)
      {
	res.expected_information.push_back(failed);
      }
      return true;
    }
    
    ViewIgResult result;
    IgRetrievalCommand command = ros_conversions::igRetrievalCommandFromMsg(req.command);
    linked_interface_->computeViewIg(command,result);
    
    BOOST_FOREACH(IgRetrievalResult& ig_res, result)
    {
      res.expected_information.push_back( ros_conversions::igRetrievalResultToMsg(ig_res) );
    }
    return true;
  }
  
  TEMPT
  bool CSCOPE::mmComputationService( ig_active_reconstruction_msgs::MapMetricCalculation::Request& req, ig_active_reconstruction_msgs::MapMetricCalculation::Response& res )
  {
    ROS_INFO("Received 'map metric computation' call.");
    if( linked_interface_ == NULL )
    {
      ig_active_reconstruction_msgs::InformationGain failed;
      failed.predicted_gain = 0;
      ResultInformation failed_status = ResultInformation::FAILED;
      failed.status = ros_conversions::resultInformationToMsg(failed_status);
      unsigned int number_of_metrics = req.metric_names.size();
      for(unsigned int i=0; i<number_of_metrics; ++i)
      {
	res.results.push_back(failed);
      }
      return true;
    }
    
    MapMetricRetrievalResultSet result;
    MapMetricRetrievalCommand command;
    BOOST_FOREACH(std::string& name, req.metric_names)
    {
      command.metric_names.push_back(name);
    }
    linked_interface_->computeMapMetric(command,result);
    
    BOOST_FOREACH(MapMetricRetrievalResult& ig_res, result)
    {
      ig_active_reconstruction_msgs::InformationGain gain;
      gain.predicted_gain = ig_res.value;
      gain.status = ros_conversions::resultInformationToMsg(ig_res.status);
      res.results.push_back(gain);
    }
    return true;
  }
  
  TEMPT
  bool CSCOPE::availableIgService( ig_active_reconstruction_msgs::StringList::Request& req, ig_active_reconstruction_msgs::StringList::Response& res )
  {
    ROS_INFO("Received 'available information gain metric' call.");
    if( linked_interface_ == NULL )
    {
      return true;
    }
    
    std::vector<MetricInfo> metric_list;
    linked_interface_->availableIgMetrics(metric_list);
    
    BOOST_FOREACH(MetricInfo& metric, metric_list)
    {
      res.names.push_back(metric.name);
      res.ids.push_back(metric.id);
    }
    return true;
  }
  
  TEMPT
  bool CSCOPE::availableMmService( ig_active_reconstruction_msgs::StringList::Request& req, ig_active_reconstruction_msgs::StringList::Response& res )
  {
    ROS_INFO("Received 'available map metric' call.");
    if( linked_interface_ == NULL )
    {
      return true;
    }
    
    std::vector<MetricInfo> metric_list;
    linked_interface_->availableMapMetrics(metric_list);
    
    BOOST_FOREACH(MetricInfo& metric, metric_list)
    {
      res.names.push_back(metric.name);
      res.ids.push_back(metric.id);
    }
    return true;
  }
  
}

}

#undef CSCOPE
#undef TEMPT