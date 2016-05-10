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


#include "ig_active_reconstruction_octomap/octomap_ig_calculator.hpp"
#include "ig_active_reconstruction/world_representation_pinhole_cam_raycaster.hpp"

namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
namespace octomap
{  
  /*! Abstract base class: Provides information gain calculation for octomap-based probabilistic volumetric world representation, implementing the frameworks communication interface. Additionally it includes a factory where the desired information gain and 
   * map metric methods can be registered.
   */
  template<class TREE_TYPE>
  class BasicRayIgCalculator: public IgCalculator<TREE_TYPE>
  {
  public:
    typedef boost::shared_ptr< BasicRayIgCalculator<TREE_TYPE> > Ptr;
    
    typedef typename IgCalculator<TREE_TYPE>::ResultInformation ResultInformation;
    typedef typename IgCalculator<TREE_TYPE>::IgRetrievalCommand IgRetrievalCommand;
    typedef typename IgCalculator<TREE_TYPE>::IgRetrievalResult IgRetrievalResult;
    typedef typename IgCalculator<TREE_TYPE>::MapMetricRetrievalCommand MapMetricRetrievalCommand;
    typedef typename IgCalculator<TREE_TYPE>::MapMetricRetrievalResult MapMetricRetrievalResult;
    typedef typename IgCalculator<TREE_TYPE>::MetricInfo MetricInfo;
    typedef typename IgCalculator<TREE_TYPE>::ViewIgRetrievalResult ViewIgRetrievalResult;
    typedef typename IgCalculator<TREE_TYPE>::MapMetricRetrievalResultSet MapMetricRetrievalResultSet;
    typedef typename IgCalculator<TREE_TYPE>::IgFactory IgFactory;
    typedef typename IgCalculator<TREE_TYPE>::MmFactory MmFactory;
    
    struct Config
    {
    public:
      /*! Constructor sets default values.
       */
      Config();
    public:
      PinholeCamRayCaster::Config ray_caster_config; //! Configuration for the pinhole ray casting module.
    };
    
  public:
    /*! Constructor.
     */
    BasicRayIgCalculator( Config config = Config() );
    
    virtual ~BasicRayIgCalculator(){};
    
    /*! Set a new configuration for raycasting.
     * @param config Raycasting configuration.
     */
    void setNewRayCastingConfig( PinholeCamRayCaster::Config& config );
    
  // Interface implementation
  public:
    /*! Calculates a set of information gains for a given view.
     * @param command Specifies which information gains have to be calculated and for which pose along with further parameters that define how the ig('s) will be collected.
     * @param output_ig (Output) Vector with the results of the information gain calculation. The indices correspond to the indices of the names in the metric_names array within the passed command.
     */
    virtual ResultInformation computeViewIg(IgRetrievalCommand& command, ViewIgRetrievalResult& output_ig);
    
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
    
    struct RayCastSettings
    {
      //double min_ray_depth; //! Minimal ray length (where it starts).
      double max_ray_depth; //! Maximal ray length.
      //double occupied_passthrough_threshold; //! If an occupied voxel's occupancy likelihood is lower than this threshold, ray casting is continued.
      //unsigned int ray_step_size; //! Voxel resolution along ray.
    };
    
  protected:
    /*! Retrieves an information for a given ray.
     * @param ray Ray which is cast.
     * @param ig_set Set of information gains to be calculated.
     * @param setting Additional ray casting settings.
     */
    void calculateIgsOnRay( RayCaster::Ray& ray, std::vector< boost::shared_ptr< InformationGain<TREE_TYPE> > >& ig_set, RayCastSettings& setting );
    
  protected:
    Config config_; //! Configuration...
    PinholeCamRayCaster ray_caster_; //! Ray caster module.
  };
}

}

}

#include "../src/code_base/octomap_basic_ray_ig_calculator.inl"