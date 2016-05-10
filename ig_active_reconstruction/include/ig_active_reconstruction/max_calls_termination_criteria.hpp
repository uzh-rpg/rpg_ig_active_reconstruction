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

#include "ig_active_reconstruction/goal_evaluation_module.hpp"

namespace ig_active_reconstruction
{
  
  /*! Simple termination criteria that returns true after a fixed
   * number of calls to isDone().
   */
  class MaxCallsTerminationCriteria: public GoalEvaluationModule
  {
  public:
    /*! Constructor.
     * @param max_calls Maximal number of calls.
     */
    MaxCallsTerminationCriteria( unsigned int max_calls );
    
    /*! Resets the goal evaluation module.
     */
    virtual void reset();
    
    /*! Returns true if the goal was reached.
     */
    virtual bool isDone();
    
  private:
    unsigned int max_calls_;
    unsigned int call_count_;
  };
  
}