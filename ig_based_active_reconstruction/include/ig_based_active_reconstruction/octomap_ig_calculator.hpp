/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of ig_based_active_reconstruction, a ROS package for...well,

ig_based_active_reconstruction is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
ig_based_active_reconstruction is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with ig_based_active_reconstruction. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "ig_based_active_reconstruction/world_representation_communication_interface.hpp"
#include "ig_based_active_reconstruction/octomap_world_representation.hpp"

namespace ig_based_active_reconstruction
{
  
namespace world_representation
{
  
namespace octomap
{  
  /*! Abstract base class: Provides information gain calculation for octomap-based probabilistic volumetric world representation, implementing the frameworks communication interface.
   */
  template<class TREE_TYPE>
  class IgCalculator: public CommunicationInterface
  {
  public:
    virtual ~IgCalculator(){};
    
    /*! Links an octomap::WorldRepresentation with the IgCalculator. 
     * Informatoin gain metrics will then be calculated on the linked world.
     * @param world the WorldRepresentation that is to be linked to the IgCalculator.
     */
    virtual void linkWorld( WorldRepresentation<TREE_TYPE>& world )=0;
    
  // Interface implementation
  public:
    /*! Calculates a set of information gains for a given view.
     * @param command Specifies which information gains have to be calculated and for which pose along with further parameters that define how the ig('s) will be collected.
     * @param output_ig (Output) Vector with the results of the information gain calculation. The indices correspond to the indices of the names in the metric_names array within the passed command.
     */
    virtual ResultInformation ComputeViewIg(IgRetrievalCommand& command, std::vector<IgRetrievalResult>& output_ig)=0;
    
    /*! Calculates a set of evaluation metrics on the complete tree.
     * @param command Specifies which metrics shall be calculated.
     */
    virtual ResultInformation computeTreeMetric(TreeMetricRetrievalCommand& command, std::vector<TreeMetricRetrievalResult>& output)=0;
    
    /*! Returns all available information gain metrics.
     * @param available_ig_metrics (output) Set of available metrics.
     */
    virtual void availableIgMetrics( std::vector<MetricInfo>& available_ig_metrics )=0;
    
    /*! Returns all available tree metrics.
     * @param available_tree_metrics (output) Set of available tree metrics.
     */
    virtual void availableTreeMetrics( std::vector<MetricInfo>& available_tree_metrics )=0;
    
  };
}

}

}