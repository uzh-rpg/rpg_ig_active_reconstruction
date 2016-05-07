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


#include <ros/ros.h>

#include <ig_active_reconstruction_ros/param_loader.hpp>
#include <ig_active_reconstruction_ros/robot_ros_server_ci.hpp>

#include "flying_gazebo_stereo_cam/robot_communication_interface.hpp"


/*! Implements a ROS node interface to a "flying" (staticly placed) gazebo stereo camera.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_interface");
  ros::NodeHandle nh;
  
  // Load parameters
  //------------------------------------------------------------------
  std::string model_name, camera_frame_name, world_frame_name;
  ros_tools::getExpParam(model_name,"model_name");
  ros_tools::getExpParam(camera_frame_name,"camera_frame_name");
  ros_tools::getExpParam(world_frame_name,"world_frame_name");
  
  using namespace flying_gazebo_stereo_cam;
  
  // Controller
  //------------------------------------------------------------------
  std::shared_ptr<Controller> controller = std::make_shared<Controller>(model_name);
  // publish tf
  controller->startTfPublisher(camera_frame_name,world_frame_name);
  
  // Iar communication interface
  //------------------------------------------------------------------
  boost::shared_ptr<CommunicationInterface> robot_interface = boost::make_shared<CommunicationInterface>(nh,controller);
  
  // Expose communication interface to ROS
  //------------------------------------------------------------------
  ig_active_reconstruction::robot::RosServerCI comm_unit(nh,robot_interface);
  
  
  // spin...
  ROS_INFO_STREAM("Flying gazebo stereo camera robot interface is setup.");
  ros::spin();
  
  return 0;
}