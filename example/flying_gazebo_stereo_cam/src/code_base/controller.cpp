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
#include "gazebo_msgs/GetModelState.h"
#include <movements/ros_movements.h>
#include <stdexcept>

namespace flying_gazebo_stereo_cam
{
  
  Controller::Controller(std::string cam_model_name)
  : cam_model_name_(cam_model_name)
  , has_moved_(false)
  , keepPublishing_(false)
  , cam_to_image_(0.5,0.5,-0.5,0.5)
  {
    
  }
  
  Controller::~Controller()
  {
    keepPublishing_=false;
    if(publisher_.joinable())
      publisher_.join();
  }
  
  bool Controller::moveTo( movements::Pose new_pose )
  {
    {
      std::lock_guard<std::mutex> guard(protector_);
      has_moved_ = true;
      current_pose_ = new_pose;
    }
    
    gazebo_msgs::SetModelState srv_call;
    srv_call.request.model_state.model_name = cam_model_name_;
    
    new_pose.orientation =  new_pose.orientation*cam_to_image_;
    srv_call.request.model_state.pose = movements::toROS( new_pose );
    
    bool response = ros::service::call( "/gazebo/set_model_state", srv_call );
    
    return srv_call.response.success;
  }
  
  movements::Pose Controller::currentPose()
  {
    gazebo_msgs::GetModelState current_state;
    current_state.request.model_name = cam_model_name_;
    
    bool response = ros::service::call( "/gazebo/get_model_state", current_state );
    
    if(!response)
      throw std::runtime_error("flying_gazebo_stereo_cam::Controller::currentPose:: Couldn't get camera pose.");
    
    movements::Pose current_pose_model = movements::fromROS(current_state.response.pose);
    movements::Pose current_pose_cam;
    current_pose_cam.orientation = cam_to_image_*current_pose_model.orientation;
    
    return current_pose_cam;
  }
  
  void Controller::startTfPublisher(std::string camera_frame_name, std::string world_frame_name)
  {
    keepPublishing_ = true;
    publisher_ = std::thread(&Controller::keepPublishing,this,camera_frame_name,world_frame_name);
  }
  
  void Controller::stopTfPublisher()
  {
    keepPublishing_=false;
  }
  
  void Controller::keepPublishing(std::string camera_frame_name, std::string world_frame_name)
  {
    while(keepPublishing_)
    {
      if(true||has_moved_)
      {
	geometry_msgs::Pose pose;
	{
	  std::lock_guard<std::mutex> guard(protector_);
	  pose = movements::toROS( currentPose()/*current_pose_*/ );
	}
	geometry_msgs::Transform pose_t;
	pose_t.translation.x = pose.position.x;
	pose_t.translation.y = pose.position.y;
	pose_t.translation.z = pose.position.z;
	pose_t.rotation.x = pose.orientation.x;
	pose_t.rotation.y = pose.orientation.y;
	pose_t.rotation.z = pose.orientation.z;
	pose_t.rotation.w = pose.orientation.w;
	
	tf::Transform cam_2_origin;
	tf::transformMsgToTF( pose_t, cam_2_origin );
	tf_broadcaster_.sendTransform(tf::StampedTransform(cam_2_origin, ros::Time::now(), world_frame_name, camera_frame_name));
      }
      ros::Duration(0.05).sleep();
    }
  }
  
}