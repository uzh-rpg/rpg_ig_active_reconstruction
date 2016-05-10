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
  
namespace views
{
  
  /*! Abstract interface definition to exchange information about the viewspace.
   */
  class CommunicationInterface
  {
  public:
    enum struct ViewSpaceStatus
    {
      OK,
      BAD,
      NONE_AVAILABLE
    };
    
    enum struct ViewSpaceUpdateResult
    {
      SUCCEEDED=0,
      FAILED,
      NOT_AVAILABLE
    };
    
  public:
    virtual ~CommunicationInterface(){};
    
    /*! Returns the view space that is available for planning.
      * @param _space pointer to the ViewSpace object that should be filled
      */
    virtual const ViewSpace& getViewSpace()=0;
    
    /*! Add a set of new views to the viewspace.
     * @param new_views New views to be added to the view space.
     */
    virtual ViewSpaceUpdateResult addViews( std::vector<View>& new_views )=0;
    
    /*! Adds a single new view to the viewspace.
     * @param new_view New view to add to the viewspace.
     */
    virtual ViewSpaceUpdateResult addView( View new_view )=0;
    
    /*! Delete a set of views from the viewspace, using their id.
     * @param view_ids Vector with the id's of the views
     * @return True if all views were successfully deleted.
     */
    virtual ViewSpaceUpdateResult deleteViews( std::vector<View::IdType>& view_ids )=0;
    
    /*! Delete a single view from the viewspace, using its id.
     * @param view_id Id of the view that shall be deleted.
     * @return True if the view was found and deleted.
     */
    virtual ViewSpaceUpdateResult deleteView( View::IdType view_id )=0;
  };
  
}

}