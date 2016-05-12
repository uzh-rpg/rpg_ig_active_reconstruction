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

#include "ig_active_reconstruction/weighted_linear_utility.hpp"

#include <thread>
#include <iostream>

namespace ig_active_reconstruction
{
  
  WeightedLinearUtility::WeightedLinearUtility( double cost_weight )
  : world_comm_unit_(nullptr)
  , robot_comm_unit_(nullptr)
  , cost_weight_(cost_weight)
  {
    
  }
  
  void WeightedLinearUtility::useInformationGain( std::string name, double weight)
  {
    information_gains_.push_back(name);
    ig_weights_.push_back(weight);
  }
  
  void WeightedLinearUtility::setCostWeight( double weight )
  {
    cost_weight_ = weight;
  }
  
  void WeightedLinearUtility::setIgRetrievalConfig( world_representation::CommunicationInterface::IgRetrievalConfig& config )
  {
    ig_retrieval_config_ = config;
  }
  
  void WeightedLinearUtility::setWorldCommUnit( boost::shared_ptr<world_representation::CommunicationInterface> world_comm_unit )
  {
    world_comm_unit_ = world_comm_unit;
  }
  
  void WeightedLinearUtility::setRobotCommUnit( boost::shared_ptr<robot::CommunicationInterface> robot_comm_unit )
  {
    robot_comm_unit_ = robot_comm_unit;
  }
  
  views::View::IdType WeightedLinearUtility::getNbv( views::ViewSpace::IdSet& id_set, boost::shared_ptr<views::ViewSpace> viewspace )
  {
    // structure to store received values
    std::vector<double> cost_vector;
    std::vector<double> ig_vector;
    
    double total_cost=0;
    double total_ig=0;
    
    world_representation::CommunicationInterface::IgRetrievalCommand command;
    command.config = ig_retrieval_config_;
    command.metric_names = information_gains_;
    
    // receive costs and igs
    for( views::View::IdType& view_id: id_set )
    {
      views::View view = viewspace->getView(view_id);
      
      robot::MovementCost cost;
      world_representation::CommunicationInterface::ViewIgResult information_gains;
      
      double cost_val = 0;
      double ig_val = 0;
      
      // cost
      if( robot_comm_unit_!=nullptr && cost_weight_!=0 )
      {
	cost = robot_comm_unit_->movementCost(view);
	
	if( cost.exception != robot::MovementCost::Exception::NONE )
	  continue; // invalid view... disregard in calculation
	else
	  cost_val = cost.cost;
      }
      
      // information gain - non multithreaded version
      /*if( world_comm_unit_!=nullptr )
      {
	command.path.clear();
	command.path.push_back( view.pose() );
	
	world_comm_unit_->computeViewIg(command,information_gains);
	for( unsigned int i= 0; i<information_gains.size(); ++i )
	{
	  if( information_gains[i].status == world_representation::CommunicationInterface::ResultInformation::SUCCEEDED )
	  {
	    //std::cout<<"\nReturned gain of metric "<<i<<":"<<information_gains[i].predicted_gain;
	    ig_val += ig_weights_[i]*information_gains[i].predicted_gain;
	  }
	}
	//std::cout<<"\nReturned total information gain is: "<<ig_val;
      }      
      total_ig += ig_val;
      ig_vector.push_back(ig_val);
      */
      total_cost += cost_val;
      cost_vector.push_back(cost_val);
    }
    
    // multithreaded information gain retrieval
    unsigned int number_of_threads = 8;
    ig_vector.resize(id_set.size(),0);
    std::vector<double> total_multitthread_ig(number_of_threads,0);
    std::vector<std::thread> threads;
    for( size_t i = 0; i<number_of_threads; ++i )
    {
      threads.push_back( std::thread(&WeightedLinearUtility::getIg,this,std::ref(ig_vector),std::ref(total_multitthread_ig[i]),command, std::ref(id_set), viewspace, i, number_of_threads ) );
    }
    for( size_t i = 0; i<number_of_threads; ++i )
    {
      threads[i].join();
      total_ig += total_multitthread_ig[i];
    }
    
    // calculate utility and choose nbv
    views::View::IdType nbv;
    double best_util = std::numeric_limits<double>::lowest();
    
    double cost_factor;
    
    if( total_ig==0 )
      total_ig=1;
    
    if( total_cost==0 )
      cost_factor=0;
    else
      cost_factor = cost_weight_/total_cost;
    
    for( unsigned int i=0; i<id_set.size(); ++i )
    {
      double utility = ig_vector[i]/total_ig - cost_factor*cost_vector[i];
      std::cout<<"\nutility of view "<<id_set[i]<<": "<<utility;
      if( utility>best_util )
      {
	best_util = utility;
	nbv = id_set[i];
      }
    }
    //std::cout<<"\nChoosing view "<<nbv<<".";
    return nbv;
  }
  
  void WeightedLinearUtility::getIg(std::vector<double>& ig_vector,double& total_ig, world_representation::CommunicationInterface::IgRetrievalCommand command, views::ViewSpace::IdSet& id_set, boost::shared_ptr<views::ViewSpace> viewspace, unsigned int base_index, unsigned int batch_size )
  {
    
    // information gain
    if( world_comm_unit_!=nullptr )
    {
      for( size_t i = base_index; i<id_set.size(); i+=batch_size )
      {	
	views::View view = viewspace->getView( id_set[i] );
	
	world_representation::CommunicationInterface::ViewIgResult information_gains;
	double ig_val = 0;
	
	command.path.clear();
	command.path.push_back( view.pose() );
	
	world_comm_unit_->computeViewIg(command,information_gains);
	
	for( unsigned int i= 0; i<information_gains.size(); ++i )
	{
	  if( information_gains[i].status == world_representation::CommunicationInterface::ResultInformation::SUCCEEDED )
	  {
	    //std::cout<<"\nReturned gain of metric "<<i<<":"<<information_gains[i].predicted_gain;
	    ig_val += ig_weights_[i]*information_gains[i].predicted_gain;
	  }
	}
	total_ig += ig_val;
	ig_vector[i] = ig_val;
      }
      return;
    }
    else
      return;
  }
    
  
}