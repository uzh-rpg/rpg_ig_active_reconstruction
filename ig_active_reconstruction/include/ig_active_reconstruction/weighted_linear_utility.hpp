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

#include "ig_active_reconstruction/utility_calculator.hpp"
#include "ig_active_reconstruction/world_representation_communication_interface.hpp"
#include "ig_active_reconstruction/robot_communication_interface.hpp"

namespace ig_active_reconstruction
{
  /*! Retrieves ig and cost for given view set, then calculates
   * a linear, but weighted combination, each normalized over the total ig and cost for all views respectively.
   * 
   * IG retrieval is in this implementation not multithreaded and thus takes some time.
   */
  class WeightedLinearUtility: public UtilityCalculator
  {    
  public:
    /*! Constructor
     * @param cost_weight Overall cost weight in the equation compared to information gains.
     */
    WeightedLinearUtility( double cost_weight = 1.0 );
    
    /*! Adds a new information gain that should be used for calculation.
     * @param name Name of the information gain to add.
     * @param weight Corresponding weight. (default=1.0)
     */
    virtual void useInformationGain( std::string name, double weight=1.0 );
    
    /*! Sets the overall cost weight.
     */
    virtual void setCostWeight( double weight );
    
    /*! Sets information gain retrieval configuratoin.
     */
    virtual void setIgRetrievalConfig( world_representation::CommunicationInterface::IgRetrievalConfig& config );
    
    /*! Sets the world representation communication interface with which the utility function corresponds.
     */
    virtual void setWorldCommUnit( boost::shared_ptr<world_representation::CommunicationInterface> world_comm_unit );
    
    /*! Sets the robot communication interface with which the utility function corresponds.
     */
    virtual void setRobotCommUnit( boost::shared_ptr<robot::CommunicationInterface> robot_comm_unit );
    
    /*! Returns the view id of the best view within the given subset of the viewspace.
     * @param id_set Id-subset of views that shall be considered.
     * @param viewspace The complete viewspace object
     */
    virtual views::View::IdType getNbv( views::ViewSpace::IdSet& id_set, boost::shared_ptr<views::ViewSpace> viewspace );  
    
  protected:
    /*! Helper function for multithreaded ig retrieval.
     * @param ig_vector (output) Vector in which the ig values will be set, must already have correct size
     * @param total_ig (output) total information gain calculated within this function
     * @param command Prebuilt command structure, only lacking the path entry
     * @param id_set Set of views for which getNbv was called.
     * @param viewspace Corresponding viewspace
     * @param base_index Which entry within the batch is processed by this function [0-(batch_size-1)]. All id's are separated into batches of size batch_size. This function will process every base_index-th of it. E.g. batch_size = 3, base_index=2: The function will process the 2th, 5th, 8th, 11th, etc entry...
     * @param batch_size Size of the batch, matches the number of spawned threads.
     */
    void getIg(std::vector<double>& ig_vector, double& total_ig, world_representation::CommunicationInterface::IgRetrievalCommand command, views::ViewSpace::IdSet& id_set, boost::shared_ptr<views::ViewSpace> viewspace, unsigned int base_index, unsigned int batch_size );
    
  protected:
    boost::shared_ptr<world_representation::CommunicationInterface> world_comm_unit_; //! Interface to world representation.
    boost::shared_ptr<robot::CommunicationInterface> robot_comm_unit_; //! Interface to robot.
    
    world_representation::CommunicationInterface::IgRetrievalConfig ig_retrieval_config_; //! Will be used for ig retrieval.
    
    std::vector< std::string > information_gains_; //! Name of the information gains to use.
    std::vector<double> ig_weights_; //! Weight of the information gains.
    double cost_weight_;
    
  };
  
}