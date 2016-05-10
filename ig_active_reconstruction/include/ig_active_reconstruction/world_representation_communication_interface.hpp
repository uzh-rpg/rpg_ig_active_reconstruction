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

#include "movements/core"

namespace ig_active_reconstruction
{
  
namespace world_representation
{

  /*! Abstract interface definition for world representation modules plus data structures used
   * for communication.
   */
  class CommunicationInterface
  {
  public:
    /*! Returned status for a specific ig metric calculation, to be treated as scoped enum.
     * Scoped enum variant for cpp03...
     */
    struct ResultInformation
    {
    public:
      enum Enum
      {
	SUCCEEDED=0, //! IG was successfully calculated.
	FAILED, //! IG could not be calculated, an error occured.
	UNKNOWN_METRIC //! The IG name was unknown and hence the IG could not be calculated
      }res;
    public:
      ResultInformation(){};
      ResultInformation(Enum a_type):res(a_type){};
      operator int(){return (int)res;};
      ResultInformation& operator=(const Enum atype){res=atype; return *this;};
      bool operator==(const Enum comp)const{return res==comp;};
      bool operator!=(const Enum comp)const{return res!=comp;};
    };
    
    /*! Result of an information gain calculation call.
     */
    struct IgRetrievalResult
    {
      ResultInformation status; //! Status.
      double predicted_gain; //! Calculated information gain if the call succeeded, undefined otherwise.
    };
    
    typedef std::vector<IgRetrievalResult> ViewIgResult;
    typedef ViewIgResult ViewIgRetrievalResult;
    typedef std::vector<ViewIgResult> ViewspaceIgResult;
    
    /*! Configuration of IgRetrievals
     */
    struct IgRetrievalConfig
    {
    public:
      /*! Describes a subwindow
       */
      struct SubWindow
      {
	double min_x_perc, max_x_perc; //! x-coordinates-window through which rays will be cast. [percentage of image width, 0.0-1.0]
	double min_y_perc, max_y_perc; //! y-coordinates-window through which rays will be cast. [percentage of image height, 0.0-1.0]
      };
      
    public:
      IgRetrievalConfig();
      
    public:
      
      double ray_resolution_x; //! How many rays are cast per pixel on the image's x-axis to obtain the information. [rays/px] Default: 1.0
      double ray_resolution_y; //! How many rays are cast per pixel on the image's y-axis to obtain the information. [rays/px] Default: 1.0
      
      SubWindow ray_window; //! Defines a subwindow of the image on which the rays shall be cast. Defaults to the complete window. Default: [0.0, 1.0] for both x- and y-coordinate windows, ie the complete image.
      
      double max_ray_depth; //! Maximal ray depth for the ig computation. [World representation units, usually m] Default: 10.0
      
    };
    
    /*! Command structure for information gain retrieval computation. The struct features a constructor that sets all members
     * to default values. See member descriptions for details.
     */
    struct IgRetrievalCommand
    {
      
    public:
      /*! Constructor loads default values.
       */
      IgRetrievalCommand();
      
    public:      
      movements::PoseVector path; //! Describes the path for which the information gain shall be calculated. Note that in the current octomap-based implementation provided with the framework this is not yet implemented: Only the first pose will be considered and no casts into the future attempted.
      std::vector<std::string> metric_names; //! Vector with the names of all metrics that shall be calculated. Only considered if metric_ids is empty.
      std::vector<unsigned int> metric_ids; //! Vector with the ids of all metrics that shall be calculated. Takes precedence over metric_names.
      IgRetrievalConfig config;
    };
    
    /*! Result of a metric calculation call.
     */
    struct MapMetricRetrievalResult
    {
      ResultInformation status; //! Status.
      double value; //! Calculated information gain if the call succeeded, undefined otherwise.
    };
    
    typedef std::vector<MapMetricRetrievalResult> MapMetricRetrievalResultSet;
    
    /*! Command to retrieve map metrics
     */
    struct MapMetricRetrievalCommand
    {
      std::vector<std::string> metric_names; //! Vector with the names of all metrics that shall be calculated.
    };
    
    /*! Struct representing metric information.
     */
    struct MetricInfo
    {
      std::string name; //! Name that uniquely identifies the metric.
      uint32_t id; //! Id that uniquely identifies the metric.
    };
    
  public:
    virtual ~CommunicationInterface(){};
    
    /*! Calculates a set of information gains for a given view.
     * @param command Specifies which information gains have to be calculated and for which pose along with further parameters that define how the ig('s) will be collected.
     * @param output_ig (Output) Vector with the results of the information gain calculation. The indices correspond to the indices of the names in the metric_names array within the passed command.
     */
    virtual ResultInformation computeViewIg(IgRetrievalCommand& command, ViewIgResult& output_ig)=0;
    
    /*! Calculates a set of evaluation metrics on the complete map.
     * @param command Specifies which metrics shall be calculated.
     */
    virtual ResultInformation computeMapMetric(MapMetricRetrievalCommand& command, MapMetricRetrievalResultSet& output)=0;
    
    /*! Returns all available information gain metrics.
     * @param available_ig_metrics (output) Set of available metrics.
     */
    virtual void availableIgMetrics( std::vector<MetricInfo>& available_ig_metrics )=0;
    
    /*! Returns all available map metrics.
     * @param available_map_metrics (output) Set of available map metrics.
     */
    virtual void availableMapMetrics( std::vector<MetricInfo>& available_map_metrics )=0;
  };
  
  
}


}