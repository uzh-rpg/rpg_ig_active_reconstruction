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
  
  /*! Abstract interface definition for goal evaluation modules, e.g. modules that evaluate if a termination criteria has been fulfilled.
   */
  class GoalEvaluationModule
  {
  public:
    virtual ~GoalEvaluationModule(){};
    
    /*! Resets the goal evaluation module.
     */
    virtual void reset()=0;
    
    /*! Returns true if the goal was reached.
     */
    virtual bool isDone()=0;
  };
  
}