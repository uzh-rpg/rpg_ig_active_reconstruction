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

#define TEMPT template<class TREE_TYPE>
#define CSCOPE BasicRayIgCalculator<TREE_TYPE>

#include <octomap/octomap_types.h>
#include <boost/foreach.hpp>

namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
namespace octomap
{ 
  TEMPT
  CSCOPE::Config::Config()
  : ray_caster_config()
  {
    
  }
  
  TEMPT
  CSCOPE::BasicRayIgCalculator( Config config )
  : config_(config)
  , ray_caster_(config.ray_caster_config)
  {
  }
  
  TEMPT
  void CSCOPE::setNewRayCastingConfig( PinholeCamRayCaster::Config& config )
  {
    ray_caster_.setConfig(config);
  }
  
  TEMPT
  typename CSCOPE::ResultInformation CSCOPE::computeViewIg(IgRetrievalCommand& command, ViewIgRetrievalResult& output_ig)
  {
    output_ig.clear();
    
    // Can't calculate ig for no given view.
    if( command.path.empty() )
    {
      IgRetrievalResult res;
      res.status = ResultInformation::FAILED;
      res.predicted_gain = 0;
      
      for( unsigned int i=0; i<( !command.metric_ids.empty()?command.metric_ids.size():command.metric_names.size() ); ++i )
      {
	output_ig.push_back(res);
      }
      return ResultInformation::FAILED;
    }

    // compute rays
    PinholeCamRayCaster::ResolutionSettings ray_caster_config;
    ray_caster_config.ray_resolution_x = command.config.ray_resolution_x;
    ray_caster_config.ray_resolution_y = command.config.ray_resolution_y;
    ray_caster_config.min_x_perc = command.config.ray_window.min_x_perc;
    ray_caster_config.min_y_perc = command.config.ray_window.min_y_perc;
    ray_caster_config.max_x_perc = command.config.ray_window.max_x_perc;
    ray_caster_config.max_y_perc = command.config.ray_window.max_y_perc;
    
    //ray_caster_.setResolution(ray_caster_config);
    boost::shared_ptr<RayCaster::RaySet> ray_set = ray_caster_.getRaySet(command.path[0]);
    
    // build ig metric set
    std::vector< boost::shared_ptr< InformationGain<TREE_TYPE> > > ig_set;
    if( !command.metric_ids.empty() )
    {
      IgRetrievalResult res;
      res.predicted_gain = 0;
      
      BOOST_FOREACH( unsigned int& id, command.metric_ids )
      {
	typename IgFactory::TypePtr ig_metric = this->ig_factory_.get(id);
	if( ig_metric==NULL )
	{
	  res.status = ResultInformation::UNKNOWN_METRIC;
	}
	else
	{
	  res.status = ResultInformation::SUCCEEDED;
	  ig_set.push_back(ig_metric);
	}
	output_ig.push_back(res);
      }
    }
    else
    {
      IgRetrievalResult res;
      res.predicted_gain = 0;
      
      BOOST_FOREACH( std::string& name, command.metric_names)
      {
	typename IgFactory::TypePtr ig_metric = this->ig_factory_.get(name);
	if( ig_metric==NULL )
	{
	  res.status = ResultInformation::UNKNOWN_METRIC;
	}
	else
	{
	  res.status = ResultInformation::SUCCEEDED;
	  ig_set.push_back(ig_metric);
	}
	output_ig.push_back(res);
      }
    }
    
    // cast rays
    RayCastSettings ray_cast_settings;
    ray_cast_settings.max_ray_depth = config_.ray_caster_config.max_ray_depth_m;//command.config.max_ray_depth;
    
    for(unsigned int i=0;i<ray_set->size();++i)
    {
      RayCaster::Ray& ray = (*ray_set)[i];
      //std::cout<<"\norigin:\n"<<ray.origin<<"\ndirection:\n"<<ray.direction<<"\n";
      BOOST_FOREACH( typename InformationGain<TREE_TYPE>::Ptr& ig, ig_set )
      {
	ig->makeReadyForNewRay();
      }
      /*if(i%100==0)
	std::cout<<"\nCalculating ray "<<i<<"/"<<ray_set->size();*/
      calculateIgsOnRay(ray,ig_set, ray_cast_settings);
    }
    
    // retrieve information gains and build output
    typename std::vector< boost::shared_ptr< InformationGain<TREE_TYPE> > >::iterator ig_it = ig_set.begin();
    BOOST_FOREACH( IgRetrievalResult& res, output_ig )
    {
      if( res.status == ResultInformation::SUCCEEDED )
      {
	res.predicted_gain = (*ig_it)->getInformation();
	std::cout<<"\nPredicted gain is: "<<res.predicted_gain;
	++ig_it;
      }
    }
    
    return ResultInformation::SUCCEEDED;
  }
  
  TEMPT
  typename CSCOPE::ResultInformation CSCOPE::computeMapMetric(MapMetricRetrievalCommand& command, MapMetricRetrievalResultSet& output)
  {
    
  }
  
  TEMPT
  void CSCOPE::availableIgMetrics( std::vector<MetricInfo>& available_ig_metrics )
  {
    typename IgFactory::Iterator it, end;
    
    for( it = this->ig_factory_.begin(), end = this->ig_factory_.end(); it!=end; ++it )
    {
      typename IgFactory::Entry& entry = *it;
      
      MetricInfo ig_metric;
      ig_metric.name = entry.name;
      ig_metric.id = entry.id;
      
      available_ig_metrics.push_back(ig_metric);
    }
  }
  
  TEMPT
  void CSCOPE::availableMapMetrics( std::vector<MetricInfo>& available_map_metrics )
  {
    typename MmFactory::Iterator it, end;
    for( it = this->mm_factory_.begin(), end = this->mm_factory_.end(); it!=end; ++it )
    {
      typename MmFactory::Entry& entry = *it;
      
      MetricInfo mm_metric;
      mm_metric.name = entry.name;
      mm_metric.id = entry.id;
      
      available_map_metrics.push_back(mm_metric);
    }
  }
  
  TEMPT
  void CSCOPE::calculateIgsOnRay( RayCaster::Ray& ray, std::vector< boost::shared_ptr< InformationGain<TREE_TYPE> > >& ig_set, RayCastSettings& setting )
  {
    using ::octomap::point3d;
    using ::octomap::KeyRay;
    using ::octomap::OcTreeKey;
    //std::cout<<"\norigin:\n"<<ray.origin<<"\ndirection:\n"<<ray.direction<<"\n";
    point3d origin( ray.origin(0),ray.origin(1),ray.origin(2) );
    point3d direction( ray.direction(0), ray.direction(1), ray.direction(2) );
    point3d end_point; // calculate endpoint (if any)
    
    double max_range = (setting.max_ray_depth>0)?setting.max_ray_depth:0.0;
    
    bool found_endpoint = this->link_.octree->castRay( origin, direction, end_point, true, max_range ); // ignore unknown cells
    
    if( !found_endpoint ) // this is necessary for occlusion based metrics but not for the others. Excluding it leads to a great speed up.
    {
      end_point = origin + direction*max_range; // use max range instead of stopping at the unknown
      found_endpoint = true;
    }
    
    if( found_endpoint )
    {
      KeyRay ray;
      this->link_.octree->computeRayKeys( origin, end_point, ray );
      for( KeyRay::iterator it = ray.begin() ; it!=ray.end(); ++it )
      {
	point3d coord = this->link_.octree->keyToCoord(*it);
	typename TREE_TYPE::NodeType* traversedVoxel = this->link_.octree->search(*it);
	BOOST_FOREACH( typename InformationGain<TREE_TYPE>::Ptr& ig, ig_set )
	{
	  ig->includeRayMeasurement( traversedVoxel );
	}
      }
      
      OcTreeKey end_key;
      if( this->link_.octree->coordToKeyChecked(end_point, end_key) )
      {
	typename TREE_TYPE::NodeType* traversedVoxel = this->link_.octree->search(end_key);
	
	BOOST_FOREACH( typename InformationGain<TREE_TYPE>::Ptr& ig, ig_set )
	{
	  ig->includeEndPointMeasurement( traversedVoxel );
	}
      }
    }
    else
    {
      BOOST_FOREACH( typename InformationGain<TREE_TYPE>::Ptr& ig, ig_set )
      {
	ig->informAboutVoidRay();
      }
    }
  }
  
}

}

}

#undef CSCOPE
#undef TEMPT