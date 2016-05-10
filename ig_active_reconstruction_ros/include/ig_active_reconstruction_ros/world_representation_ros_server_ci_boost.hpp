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
#include "ig_active_reconstruction/world_representation_communication_interface.hpp"

#include "ig_active_reconstruction_msgs/InformationGainCalculation.h"
#include "ig_active_reconstruction_msgs/MapMetricCalculation.h"
#include "ig_active_reconstruction_msgs/StringList.h"

namespace ig_active_reconstruction
{
  
namespace world_representation
{

  /*! ROS server implementation of a world_representation::CommunicationInterface. Receives ROS service calls and forwards them to the linked interface.
   */
  class RosServerCI: public CommunicationInterface
  {
  public:
    /*! Constructor
     * @param nh ROS node handle defines the namespace in which ROS communication will be carried out.
     * @param linked_interface (optional) directly add the interface that is linked internally (to which requests are forwarded.
     */
    RosServerCI( ros::NodeHandle nh, std::shared_ptr<CommunicationInterface> linked_interface = nullptr );
    
    virtual ~RosServerCI(){};
    
    /*! Calculates a set of information gains for a given view.
     * @param command Specifies which information gains have to be calculated and for which pose along with further parameters that define how the ig('s) will be collected.
     * @param output_ig (Output) Vector with the results of the information gain calculation. The indices correspond to the indices of the names in the metric_names array within the passed command.
     */
    virtual ResultInformation computeViewIg(IgRetrievalCommand& command, ViewIgResult& output_ig);
    
    /*! Calculates a set of evaluation metrics on the complete map.
     * @param command Specifies which metrics shall be calculated.
     */
    virtual ResultInformation computeMapMetric(MapMetricRetrievalCommand& command, MapMetricRetrievalResultSet& output);
    
    /*! Returns all available information gain metrics.
     * @param available_ig_metrics (output) Set of available metrics.
     */
    virtual void availableIgMetrics( std::vector<MetricInfo>& available_ig_metrics );
    
    /*! Returns all available map metrics.
     * @param available_map_metrics (output) Set of available map metrics.
     */
    virtual void availableMapMetrics( std::vector<MetricInfo>& available_map_metrics );
    
  protected:
    bool igComputationService( ig_active_reconstruction_msgs::InformationGainCalculation::Request& req, ig_active_reconstruction_msgs::InformationGainCalculation::Response& res );
    bool mmComputationService( ig_active_reconstruction_msgs::MapMetricCalculation::Request& req, ig_active_reconstruction_msgs::MapMetricCalculation::Response& res );
    bool availableIgService( ig_active_reconstruction_msgs::StringList::Request& req, ig_active_reconstruction_msgs::StringList::Response& res );
    bool availableMmService( ig_active_reconstruction_msgs::StringList::Request& req, ig_active_reconstruction_msgs::StringList::Response& res );
    
  protected:
    ros::NodeHandle nh_;
    
    std::shared_ptr<CommunicationInterface> linked_interface_; //! Linked interface.
    
    ros::ServiceServer view_ig_computation_;
    ros::ServiceServer map_metric_computation_;
    ros::ServiceServer available_ig_receiver_;
    ros::ServiceServer available_mm_receiver_;
  };
  
  
}


}