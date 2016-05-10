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

#include "ig_active_reconstruction/views_communication_interface.hpp"

namespace ig_active_reconstruction
{
  
namespace views
{
  
  /*! Simple view space module: Views can be added and deleted and the viewspace can be loaded or saved to file,
   * but no other dynamic functionality is provided.
   */
  class SimpleViewSpaceModule: public CommunicationInterface
  {
  public:
    /*! Constructor, optionally directly loads the viewspace from file.
     * @param file_source Path to the file.
     */
    SimpleViewSpaceModule( std::string file_source = "" );
    
    virtual ~SimpleViewSpaceModule(){};
    
    /*! Loads the viewspace from file
     * 
     * Format (first number of views in the file, then each view represented by its position and a quaternion for its orientation):
     * Nr_of_views
     * pos_1.x pos_1.y pos_1.z orientation_1.x orientation_1.y orientation_1.z orientation_1.w
     * pos_2.x pos_2.y pos_2.z orientation_2.x orientation_2.y orientation_2.z orientation_2.w
     * (...)
     * 
     * @param path Path to the file.
     */
    void loadFromFile( std::string path );
    
    /*! Saves the current viewspace to file (view position and orientation only, no additional information)
     * 
     * Format (first number of views in the file, then each view represented by its position and a quaternion for its orientation):
     * Nr_of_views
     * pos_1.x pos_1.y pos_1.z orientation_1.x orientation_1.y orientation_1.z orientation_1.w
     * pos_2.x pos_2.y pos_2.z orientation_2.x orientation_2.y orientation_2.z orientation_2.w
     * (...)
     * 
     * @param path Path to the file.
     */
    void saveToFile( std::string filename );
    
    /*! Returns the view space that is available for planning.
      * @param _space pointer to the ViewSpace object that should be filled
      */
    virtual const ViewSpace& getViewSpace();
    
    /*! Add a set of new views to the viewspace.
     * @param new_views New views to be added to the view space.
     */
    virtual ViewSpaceUpdateResult addViews( std::vector<View>& new_views );
    
    /*! Adds a single new view to the viewspace.
     * @param new_view New view to add to the viewspace.
     */
    virtual ViewSpaceUpdateResult addView( View new_view );
    
    /*! Delete a set of views from the viewspace, using their id.
     * @param view_ids Vector with the id's of the views
     * @return True if all views were successfully deleted.
     */
    virtual ViewSpaceUpdateResult deleteViews( std::vector<View::IdType>& view_ids );
    
    /*! Delete a single view from the viewspace, using its id.
     * @param view_id Id of the view that shall be deleted.
     * @return True if the view was found and deleted.
     */
    virtual ViewSpaceUpdateResult deleteView( View::IdType view_id );
    
  public:
    ViewSpace viewspace_; //! The internal viewspace.
    
  };
}

}