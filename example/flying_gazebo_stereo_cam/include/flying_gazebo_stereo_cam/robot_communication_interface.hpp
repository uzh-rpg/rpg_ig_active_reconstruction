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

#include <ros/ros.h>
#include "ig_active_reconstruction/robot_communication_interface.hpp"
#include "flying_gazebo_stereo_cam/controller.hpp"
#include "flying_gazebo_stereo_cam/pcl_rerouter.hpp"

namespace flying_gazebo_stereo_cam
{
  
  class CommunicationInterface: public ig_active_reconstruction::robot::CommunicationInterface
  {
  public:
    typedef ig_active_reconstruction::views::View View;
    typedef ig_active_reconstruction::robot::MovementCost MovementCost;
    
  public:
    /*! Constructor
     * @param nh Sets the ros node handle used for data subscription and publication.
     * @param controller Sets the controller of the camera that will be moved.
     * @param in_name Input name (Where pcl input is retrieved, relative to node namespace)
     * @param out_name How outputs (srv/topic) are advertised (relative to node namespace)
     */
    CommunicationInterface( ros::NodeHandle nh, std::shared_ptr<Controller> controller, std::string in_name="in", std::string out_name="out" );
    
    /*! returns the current view */
    virtual View getCurrentView();
    
    /*! Commands robot to retrieve new data.
    * @return information about what happened (data received, receival failed )
    */
    virtual ReceptionInfo retrieveData();
    
    /*! Returns the cost to move from the current view to the indicated view
    * @param target_view the next view
    * @return cost to move to that view: Simple distance
    */
    virtual MovementCost movementCost( View& target_view );
    
    /*! returns the cost to move from start view to target view
    * @param start_view the start view
    * @param target_view the target view
    * @param fill_additional_information if true then the different parts of the cost will be included in the additional fields as well
    * @return cost for the movement: Simple distance.
    */
    virtual MovementCost movementCost( View& start_view, View& target_view, bool fill_additional_information  );
    
    /*! Tells the robot to get the camera to a new view
    * @param target_view where to move to
    * @return false if the operation failed
    */
    virtual bool moveTo( View& target_view );
    
  private:
    std::shared_ptr<Controller> cam_controller_; //! For movements etc.
    ros_tools::PclRerouter pcl_rerouter_; //! Since the gazebo stereo camera outputs a continuous stream of data but we are only interested in on dataset at a particular time, data retrieval consists in rerouting one data packet to the correct output where it is processed further.
  };
  
}