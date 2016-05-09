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

#ifndef IG_ACTIVE_RECONSTRUCTION_VIEW_SPACE_H_
#define IG_ACTIVE_RECONSTRUCTION_VIEW_SPACE_H_

#include <Eigen/StdVector>
#include "ig_active_reconstruction/view.hpp"

#include <map> //TODO use unordered map as soon as cpp03 is not needed anymore
#include <iterator>

namespace ig_active_reconstruction
{
  
namespace views
{

/*! Container class for possible camera orientations (views).
 * 
 * TODO: Change internal implementation details to speed up insertions and deletions.
 */
class ViewSpace
{
public:
  //typedef std::map<View::IdType, View >::iterator Iterator;
  class Iterator; // forward declaration for bidirectional iterator type to View
  //typedef std::map<View::IdType, View >::const_iterator ConstIterator;
  class ConstIterator; // forward declaration for bidirectional iterator type to const View
  
  typedef std::vector<View::IdType> IdSet;
  
public:
  
  ViewSpace();
    
  /*! returns all view points in the view space as a vector 
   */
  std::vector<View, Eigen::aligned_allocator<View> > getViewSpace();
  
  /*! returns indexes of all view points in the view space as a vector that are reachable, are not "bad" and (optionally) have never been visited.
   * @param ignore_visited whether already visited views are left out of the "good viewspace" or not
   */
  void getGoodViewSpace( IdSet& out, bool ignore_visited=true );
  
  /*!
   * returns the view corresponding to index _index.
   * @throws std::out_of_range if _index is invalid
   */
  View getView( View::IdType index );
  
  /*! Removes the given view from the view space if it is found.
   * TODO: Current implementation is rather slow.
   * 
   * @param index Index of the view.
   * @return True if it was found (and therefore removed).
   */
  bool deleteView( View::IdType index );
  
  /*! Removes a set of views from the view space if they are found.
   * TODO: Current implementation is slow.
   * 
   * @param index_set Set of indices of views that shall be deleted.
   * @return True if all views were found (and deleted). If it is false, some may still have been deleted.
   */
  bool deleteViews( std::vector<View::IdType>& index_set );
  
  /*!
   * returns how many times the view with index _index has already been visited
   * @param _index view index
   * @return How many times the view has been visited. If the view id is invalid, zero is returned.
   */
  unsigned int timesVisited( View::IdType index );
  
  void setBad( View::IdType index );
  void setGood( View::IdType index );
  void setVisited( View::IdType index );
  void setUnReachable( View::IdType index );
  void setReachable( View::IdType index );
  
  /*! adds a new view point to the view space
   * @param _new_vp the new view point
   */
  void push_back( View new_vp );
  
  /*! returns the view closest to the position passed, ignoring orientation. If more than one views in the view space have the same distance, the first found is returned
   * @param _view the view for which the closest other view is sought
   * @throws std::runtime_error if the view space is empty
   */
  View getAClosestNeighbour( View& view );
  
  /*!
   * return the size of the view space
   */
  unsigned int size();
  
  /*! returns all views within a certain range (distance) of another view
   * @param _reference_view view from which the distances are calculated
   * @param _distance the distance (<=)
   * @param _sub_space vector to fill with the results
   */
  void getViewsInRange( View& reference_view, double distance, std::vector<View, Eigen::aligned_allocator<View> >& sub_space );
  
  /*!
   * saves the poses in the view space to file
   */
  void saveToFile( std::string filename );
  
  /*! Loads the viewspace from file
   * 
   * Format (first number of views in the file, then each view represented by its position and a quaternion for its orientation):
   * Nr_of_views
   * pos_1.x pos_1.y pos_1.z orientation_1.x orientation_1.y orientation_1.zorientation_1.w
   * pos_2.x pos_2.y pos_2.z orientation_2.x orientation_2.y orientation_2.z orientation_2.w
   * (...)
   * 
   * @param filename Path to the file.    
   */
  void loadFromFile( std::string filename );
  
  /*! Providing means to iterate over view space
   */
  Iterator begin();
  ConstIterator begin() const;
  
  /*! Providing means to iterate over view space
   */
  Iterator end();
  ConstIterator end() const;
  
  /*! Whether the viewspace is empty or not.
   */
  bool empty() const;
  
protected:
  /*! Recalculates the internal index map.
   */
  void recalculateIndexMap();
  
private:
  std::vector<View, Eigen::aligned_allocator<View> > view_space_; //! Actual storage, used for iterations.
  std::map<View::IdType, View > views_index_map_; //! For access by index.
};

/*! Bidirectional iterator
 */
class ViewSpace::Iterator: public std::iterator<std::bidirectional_iterator_tag, View>
{
public:
  typedef std::map<View::IdType, View >::iterator InternalIteratorType;
  
public:
  Iterator();
  Iterator(InternalIteratorType it);
  
  bool operator==(const Iterator&) const;
  bool operator!=(const Iterator&) const;
  
  View& operator*() const;
  View* operator->() const;
  
  Iterator& operator++();
  Iterator operator++(int);
  
  Iterator& operator--();
  Iterator operator--(int);
  
private:
  InternalIteratorType it_;
};

/*! Bidirectional iterator
 */
class ViewSpace::ConstIterator
{
public:
  typedef std::map<View::IdType, View >::const_iterator InternalIteratorType;
  
public:
  ConstIterator();
  ConstIterator(InternalIteratorType it);
  
  bool operator==(const ConstIterator&) const;
  bool operator!=(const ConstIterator&) const;
  
  const View& operator*() const;
  const View* operator->() const;
  
  ConstIterator& operator++();
  ConstIterator operator++(int);
  
  ConstIterator& operator--();
  ConstIterator operator--(int);
  
private:
  InternalIteratorType it_;
};

}

}

#endif