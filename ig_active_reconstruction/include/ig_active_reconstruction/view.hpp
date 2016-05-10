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

#ifndef IG_BASED_ACTIVE_RECONSTRUCTION_VIEW_H_
#define IG_BASED_ACTIVE_RECONSTRUCTION_VIEW_H_

#include "movements/core"

namespace ig_active_reconstruction
{
  
namespace views
{

/*! Represents one possible view in space with associated information.
 */
class View
{
public:
  class ViewInfo;
  
  typedef uint64_t IdType;
  
public:
  /*! Empty constructor
   */
  View();
  
  /*! Constructor with given view source frame
   */
  View( std::string source_frame );
  
  /*! Constructor with given id. (This is for communication purposes, if the view already exists and should keep its id).
   */
  View( IdType id );
  
  /*! grants access to the pose */
  movements::Pose& pose();
  const movements::Pose& pose() const;
  
  /*! grants access to the source frame description */
  std::string& sourceFrame();
  const std::string& sourceFrame() const;
  
  /*! whether this view is reachable by the robot
   */
  bool& reachable();
  const bool& reachable() const;
  
  /*! how many times this view has already been visited */
  unsigned int& timesVisited();
  const unsigned int& timesVisited() const;
  
  /*! this view has been bad, or not
   */
  bool& bad();
  const bool& bad() const;
  
  /*! Whether this view is part of a viewspace or not.
   */
  bool& nonViewSpace();
  const bool& nonViewSpace() const;
  
  /*! get the associated data */
  boost::shared_ptr<ViewInfo>& associatedData();
  
  /*! Returns the index of the view.
   */
  IdType index() const;
  
  /*! Returns names of additional fields
   */
  std::vector<std::string>& additionalFieldsNames();
  const std::vector<std::string>& additionalFieldsNames() const;
  
  /*! Returns values of additional fields.
   */
  std::vector<double>& additionalFieldsValues();
  const std::vector<double>& additionalFieldsValues() const;
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
private:
  
  IdType index_; //! Unique view index per process, increased by one each time a new view is created.
  static IdType runningIndex_; //! The index of the next newly created view.
  
  movements::Pose pose_; //! the pose...
  std::string source_frame_; //! pose is relative to this frame, that is the transform that could be used to transform a point in the pose frame into one in the source frame
  boost::shared_ptr<ViewInfo> associated_data_; //! interface that gives possibility for a robot to store robot specific data that is associated with the pose, e.g. for the Youbot some IK calculations can be made beforehand but the robot needs to know which data implements a given view
  std::vector<std::string> additional_fields_names_;
  std::vector<double> additional_fields_values_;
  bool is_reachable_; //! whether the view is reachable or not
  bool is_bad_; //! marks a bad view for whatever reason (e.g. data reception failed here or the like)
  unsigned int visited_; //! how many times this view has been visited
  bool non_viewspace_; //! True if this view is not part of a viewspace.
};

/*! Interface for robot specific information that is associated with a given view */
class View::ViewInfo
{
public:
  ViewInfo(){};
  
  /*! returns the type of the implementing class */
  virtual std::string type()=0;
};

}

} 

std::ostream& operator<<(std::ostream& _out, ig_active_reconstruction::views::View& _view );


#endif