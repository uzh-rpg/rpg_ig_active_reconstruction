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

#include "ros/ros.h"
#include "ig_active_reconstruction/views_communication_interface.hpp"

#include "ig_active_reconstruction_msgs/DeleteViews.h"
#include "ig_active_reconstruction_msgs/ViewSpaceRequest.h"
#include "ig_active_reconstruction_msgs/ViewSpaceUpdate.h"

namespace ig_active_reconstruction
{
  
namespace views
{
  
  /*! ROS server implementation of a views::CommunicationInterface. Receives ROS service calls and forwards them to the linked interface.
   */
  class RosServerCI: public CommunicationInterface
  {
  public:
    /*! Constructor
     * @param nh ROS node handle defines the namespace in which ROS communication will be carried out.
     * @param linked_interface (optional) directly add the interface that is linked internally (to which requests are forwarded.
     */
    RosServerCI( ros::NodeHandle nh, boost::shared_ptr<CommunicationInterface> linked_interface = nullptr );
    
    /*! Returns the viewspace.
      */
    virtual const ViewSpace & getViewSpace();
    
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
    
  protected:
    bool viewspaceService( ig_active_reconstruction_msgs::ViewSpaceRequest::Request& req, ig_active_reconstruction_msgs::ViewSpaceRequest::Response& res );
    
    bool viewsAdderService( ig_active_reconstruction_msgs::ViewSpaceUpdate::Request& req, ig_active_reconstruction_msgs::ViewSpaceUpdate::Response& res );
    
    bool viewsDeleterService( ig_active_reconstruction_msgs::DeleteViews::Request& req, ig_active_reconstruction_msgs::DeleteViews::Response& res );
    
  protected:
    ros::NodeHandle nh_;
    
    boost::shared_ptr<CommunicationInterface> linked_interface_; //! Linked interface.
    
    ros::ServiceServer viewspace_service_;
    ros::ServiceServer views_adder_service_;
    ros::ServiceServer views_deleter_service_;
  };
  
}

}