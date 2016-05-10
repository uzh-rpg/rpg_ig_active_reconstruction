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

#include "ig_active_reconstruction/view_space.hpp"

namespace ig_active_reconstruction
{
  
  /*! Interface definition for utility function calculators.
   */
  class UtilityCalculator
  {    
  public:
    virtual ~UtilityCalculator(){};
    
    /*! Returns the view id of the best view within the given subset of the viewspace.
     * @param id_set Id-subset of views that shall be considered.
     * @param viewspace The complete viewspace object
     */
    virtual views::View::IdType getNbv( views::ViewSpace::IdSet& id_set, boost::shared_ptr<views::ViewSpace> viewspace )=0;    
  };
  
}