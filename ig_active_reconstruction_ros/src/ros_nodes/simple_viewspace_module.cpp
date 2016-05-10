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

#include <ros/ros.h>

#include "ig_active_reconstruction_ros/param_loader.hpp"
#include "ig_active_reconstruction/views_simple_view_space_module.hpp"
#include "ig_active_reconstruction_ros/views_ros_server_ci.hpp"

/*! Implements a ROS node holding a SimpleViewSpace module, loading the (possibly only initial) viewspace from file.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_viewspace_module");
  ros::NodeHandle nh;
  
  namespace iar = ig_active_reconstruction;
  
  // Load configuration
  //-----------------------------------------------------------------------------------------
  std::string viewspace_file_path;
  ros_tools::getExpParam(viewspace_file_path,"viewspace_file_path");
  
  // Instantiate viewspace module
  //-----------------------------------------------------------------------------------------
  boost::shared_ptr<iar::views::CommunicationInterface> viewspace_module = boost::make_shared<iar::views::SimpleViewSpaceModule>(viewspace_file_path);
  
  // Expose the viewspace module to ROS
  //-----------------------------------------------------------------------------------------
  iar::views::RosServerCI comm_unit(nh,viewspace_module);
  
  
  // spin...
  ROS_INFO("simple_viewspace_module is ready");
  ros::spin();
  
  return 0;
}