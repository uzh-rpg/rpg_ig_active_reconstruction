/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of dense_reconstruction, a ROS package for...well,

dense_reconstruction is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
dense_reconstruction is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with dense_reconstruction. If not, see <http://www.gnu.org/licenses/>.
*/

#include "dense_reconstruction/cost_matrix.h"
#include "dense_reconstruction/robot_planning_interface.h"


namespace dense_reconstruction
{

CostMatrix::CostMatrix()
{
  cost_retriever_ = nh_.serviceClient<dense_reconstruction::MovementCostCalculation>("/dense_reconstruction/robot_interface/movement_cost");
}

CostMatrix::CostMatrix( ViewSpace& _view_space )
{
  cost_retriever_ = nh_.serviceClient<dense_reconstruction::MovementCostCalculation>("/dense_reconstruction/robot_interface/movement_cost");
  createCostPairs( _view_space );
}

bool CostMatrix::movementCost( RobotPlanningInterface::MovementCost& _output, View& _start_view, View& _target_view );
{
  MovementCostCalculation request;
  request.request.start_view = _start_view.toMsg();
  request.request.target_view = _target_view.toMsg();
  request.request.additional_information = true;
  
  bool response = cost_retriever_.call(request);
  
  if( response )
  {
    _output.fromMsg( request.response.movement_cost );
  }
  
  return response;
}

bool CostMatrix::createCostPairs( ViewSpace& _view_space )
{
  unsigned int view_space_size_ = _view_space.size();
  
  for( unsigned int i=0; i<view_space_size_-1; i++ )
  {
    View start = _view_space.getView(i);
    for( unsigned int j=i+1; j<view_space_size_; j++ )
    {
      View target = _view_space.getView(j);
      
      RobotPlanningInterface::MovementCost cost_description;
      
      if( !movementCost(cost_description,start,target) )
      {
	if( cost_description==RobotPlanningInterface::MovementCost::NONE )
	{
	  cost_matrix_[i][j] = cost_description.cost;
	}
	else
	{
	  ROS_ERROR_STREAM("CostMatrix: Exception occured for movement cost calculation. The flag returned was '"<<cost_description.exception<<"'. Cost matrix calculation failed.");
	  return false
	}
      }
      else
      {
	ROS_ERROR("CostMatrix: Could not retrieve cost for all combinations.");
	return false;
      }
    }
  }
  return true;
}

void CostMatrix::toFile( std::string _path_name )
{
  std::ofstream out( _filename, std::ofstream::trunc );
  
  for( unsigned int i=0; i<view_space_size_; i++ )
  {
    for( unsigned int j=0; j<view_space_size_; j++ )
    {
      if( i<j )
      {
	out<<cost_matrix_[i][j];
      }
      else if( j<i )
      {
	out<<cost_matrix_[j][i];
      }
      else
	out<<-1;
      
      out<<" ";
    }
    out<<"\n";
  }
}

}