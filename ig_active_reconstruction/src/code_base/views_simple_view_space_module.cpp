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


#include "ig_active_reconstruction/views_simple_view_space_module.hpp"


namespace ig_active_reconstruction
{
  
namespace views
{
  SimpleViewSpaceModule::SimpleViewSpaceModule( std::string file_source )
  {
    if( file_source!="" )
    {
      loadFromFile(file_source);
    }
  }
  
  void SimpleViewSpaceModule::loadFromFile( std::string path )
  {
    viewspace_.loadFromFile(path);
  }
  
  void SimpleViewSpaceModule::saveToFile( std::string filename )
  {
    viewspace_.saveToFile(filename);
  }
  
  const ViewSpace & SimpleViewSpaceModule::getViewSpace()
  {
    return viewspace_;
  }
  
  SimpleViewSpaceModule::ViewSpaceUpdateResult SimpleViewSpaceModule::addViews( std::vector<View>& new_views )
  {
    for( View& view: new_views )
    {
      viewspace_.push_back(view);
    }
    return ViewSpaceUpdateResult::SUCCEEDED;
  }
  
  SimpleViewSpaceModule::ViewSpaceUpdateResult SimpleViewSpaceModule::addView( View new_view )
  {
    viewspace_.push_back(new_view);
    return ViewSpaceUpdateResult::SUCCEEDED;
  }
  
  SimpleViewSpaceModule::ViewSpaceUpdateResult SimpleViewSpaceModule::deleteViews( std::vector<View::IdType>& view_ids )
  {
    if( viewspace_.deleteViews(view_ids) )
      return ViewSpaceUpdateResult::SUCCEEDED;
    else
      ViewSpaceUpdateResult::FAILED;
  }
  
  SimpleViewSpaceModule::ViewSpaceUpdateResult SimpleViewSpaceModule::deleteView( View::IdType view_id )
  {
    if( viewspace_.deleteView(view_id) )
      return ViewSpaceUpdateResult::SUCCEEDED;
    else
      ViewSpaceUpdateResult::FAILED;
  }
  
}

}
