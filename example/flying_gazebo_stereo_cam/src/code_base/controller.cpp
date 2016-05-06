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

#include "flying_gazebo_stereo_cam/controller.hpp"

#include "ros/ros.h"
#include "gazebo_msgs/SetModelState.h"
#include <movements/ros_movements.h>

namespace flying_gazebo_stereo_cam
{
  
  Controller::Controller(std::string cam_model_name)
  : cam_model_name_(cam_model_name)
  , cam_to_image_(0.5,0.5,-0.5,0.5)
  {
    
  }
  
  bool Controller::moveTo( movements::Pose new_pose )
  {
    gazebo_msgs::SetModelState srv_call;
    srv_call.request.model_state.model_name = cam_model_name_;
    
    new_pose.orientation =  new_pose.orientation*cam_to_image_;
    srv_call.request.model_state.pose = movements::toROS( new_pose );
    
    bool response = ros::service::call( "/gazebo/set_model_state", srv_call );
    
    return response;
  }
  
}