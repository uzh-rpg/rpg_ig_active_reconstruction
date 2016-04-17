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

#pragma once

#include "dense_reconstruction/view.h"
#include <Eigen/StdVector>
#include "dense_reconstruction/ViewSpaceMsg.h"

namespace dense_reconstruction
{

/** container for possible camera orientations (views)
 *@TODO ideas/suggestions: iterator to iterate over view space, get subsets of space
 */
class ViewSpace
{
public:
  
  ViewSpace();
  
  /** set view space from message, overwrites all previous data */
  void fromMsg( const ViewSpaceMsg& _msg );
  
  /** creates a view space msg with the content of the view space
   */
  ViewSpaceMsg toMsg();
  
  /** returns all view points in the view space as a vector 
   */
  std::vector<View, Eigen::aligned_allocator<View> > getViewSpace();
  
  /** returns indexes of all view points in the view space as a vector that are reachable, have never been visited and are not "bad"
   * @param _ignore_visited whether already visited views are left out of the "good viewspace" or not
   */
  void getGoodViewSpace( std::vector<unsigned int>& _out, bool _ignore_visited=true );
  
  /**
   * returns the view corresponding to index _index.
   * @throws std::invalid_argument if _index is invalid
   */
  View getView( unsigned int _index );
  
  /**
   * returns how many times the view with index _index has already been visited
   * @param _index view index
   */
  unsigned int timesVisited( unsigned int _index );
  
  void setBad( unsigned int _index );
  void setGood( unsigned int _index );
  void setVisited( unsigned int _index );
  void setUnReachable( unsigned int _index );
  void setReachable( unsigned int _index );
  
  /** adds a new view point to the view space
   * @param _new_vp the new view point
   */
  void push_back( View _new_vp );
  
  /** returns the view closest to the position passed, ignoring orientation. If more than one views in the view space have the same distance, the first found is returned
   * @param _view the view for which the closest other view is sought
   * @throws std::runtime_error if the view space is empty
   */
  View getAClosestNeighbour( View& _view );
  
  /**
   * return the size of the view space
   */
  unsigned int size();
  
  /** returns all views within a certain range (distance) of another view
   * @param _reference_view view from which the distances are calculated
   * @param _distance the distance (<=)
   * @param _sub_space vector to fill with the results
   */
  void getViewsInRange( View& _reference_view, double _distance, std::vector<View, Eigen::aligned_allocator<View> >& _sub_space );
  
  /**
   * saves the poses in the view space to file
   */
  void saveToFile( std::string _filename );
  
  /**
   * loads poses from file
   */
  void loadFromFile( std::string _filename );
  
private:
  std::vector<View, Eigen::aligned_allocator<View> > view_space_;
};

}