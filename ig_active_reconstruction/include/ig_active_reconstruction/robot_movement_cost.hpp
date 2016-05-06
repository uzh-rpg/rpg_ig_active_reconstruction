/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of ig_active_reconstruction, a ROS package for...well,

ig_active_reconstruction is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
ig_active_reconstruction is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with ig_active_reconstruction. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

namespace ig_active_reconstruction
{
  
namespace robot
{

  /*! The MovementCost class describes the cost to move the sensor between different positions.
   * All its members are currently public.
   */
  struct MovementCost
  {
  public:
    enum struct Exception
    {
      NONE, 
      COST_UNKNOWN, 
      INFINITE_COST, 
      INVALID_STATE, 
      INVALID_TARGET_STATE, 
      INVALID_START_STATE,
      RECEPTION_FAILED
    };
    
  public:
    
    MovementCost():exception(Exception::NONE){};
    
    
    
  public:
    double cost; // !keeping it simple
    //! possible exceptions:: INFINITE_COST: do not move to target view, INVALID_STATE: robot is in state which somehow prevents it from calculating a cost, but the movement might be possible
    Exception exception;
    std::vector<std::string> additional_field_names; //! names for additional information fields (optional)
    std::vector<double> additional_fields_values; //! values corresponding to the description in additional_fiel
    };
  
}

}