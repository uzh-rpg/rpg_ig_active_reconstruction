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

#include "dense_reconstruction/remode_data_retriever.h"

namespace dense_reconstruction
{
  
std_msgs::String RemodeDataRetriever::START_RECONSTRUCTION;
std_msgs::String RemodeDataRetriever::STOP_AND_SMOOTH;
std_msgs::String RemodeDataRetriever::SET_IDLE;


RemodeDataRetriever::RemodeDataRetriever( YoubotPlanner* _robot_interface, std::string _youbot_interface_namespace )
  :robot_interface_(_robot_interface)
{
  START_RECONSTRUCTION.data="SET_NEXT_FRAME_IS_REFERENCE";
  STOP_AND_SMOOTH.data="SMOOTH_AND_STOP_UPDATING";
  SET_IDLE.data="SET_IDLE";
  
  if( !robot_interface_->ros_node_->getParam("/"+_youbot_interface_namespace+"/initialization/remode/scan_radius",scanning_radius_) )
  {
    ROS_WARN_STREAM("RemodeDataRetriever:: No scan radius was found on parameter server ('"<<"/"+_youbot_interface_namespace+"/initialization/remode/scan_radius"<<"'), the default (0.05m) will be used.");
    scanning_radius_=0.05;
  }
  std::string remode_control_topic, octomap_topic, pcl_topic;
  if( !robot_interface_->ros_node_->getParam("/"+_youbot_interface_namespace+"/initialization/remode/control_topic",remode_control_topic) )
  {
    ROS_WARN_STREAM("RemodeDataRetriever:: No control topic was found on parameter server ('"<<"/"+_youbot_interface_namespace+"/initialization/remode/control_topic"<<"'), the default ('/remode/command') will be used.");
    remode_control_topic="/remode/command";
  }
  if( !robot_interface_->ros_node_->getParam("/"+_youbot_interface_namespace+"/initialization/remode/octomap_topic",octomap_topic) )
  {
    ROS_WARN_STREAM("RemodeDataRetriever:: No octomap topic was found on parameter server ('"<<"/"+_youbot_interface_namespace+"/initialization/remode/octomap_topic"<<"'), the default ('/octomap_full') will be used.");
    octomap_topic="/octomap_full";
  }
  if( !robot_interface_->ros_node_->getParam("/"+_youbot_interface_namespace+"/initialization/remode/pcl_topic",pcl_topic) )
  {
    ROS_WARN_STREAM("RemodeDataRetriever:: No pointcloud topic was found on parameter server ('"<<"/"+_youbot_interface_namespace+"/initialization/remode/pcl_topic"<<"'), the default ('/remode/pointcloud_single') will be used.");
    octomap_topic="/remode/pointcloud_single";
  }
  double max_wait_time;
  if( !robot_interface_->ros_node_->getParam("/"+_youbot_interface_namespace+"/initialization/remode/max_wait_time",max_wait_time) )
  {
    ROS_WARN_STREAM("RemodeDataRetriever:: No max_wait_time was found on parameter server ('"<<"/"+_youbot_interface_namespace+"/initialization/remode/max_wait_time"<<"'), the default (3s) will be used.");
    max_remode_wait_time_=ros::Duration(3.0); // 3 seconds
  }
  else
  {
    max_remode_wait_time_=ros::Duration(max_wait_time); // [s]
  }
  
  remode_has_published_=false;
  octomap_has_published_ = false;
  remode_commander_ = robot_interface_->ros_node_->advertise<std_msgs::String>( remode_control_topic, 1 );
  octomap_topic_subsriber_ = robot_interface_->ros_node_->subscribe( octomap_topic,1, &dense_reconstruction::RemodeDataRetriever::octomapCallback, this );
  remode_topic_subscriber_ = robot_interface_->ros_node_->subscribe( pcl_topic,1, &dense_reconstruction::RemodeDataRetriever::remodeCallback, this );
}

std::string RemodeDataRetriever::movementConfigurationDescription()
{
  std::stringstream name;
  name<<"InOutSpiral_"<<scanning_radius_;
  return name.str();
}

RobotPlanningInterface::ReceiveInfo RemodeDataRetriever::retrieveData()
{
  if( robot_interface_->current_view_==nullptr )
  {
    // TODO in unknown pose: replan!
    ROS_WARN("YoubotPlanner::RemodeDataRetriever::retrieveData::Cannot retrieve data because current view is unknown and thus no scanning movement is available. Retrieving data failed.");
    return RobotPlanningInterface::RECEPTION_FAILED;
  }
  
  moveit::planning_interface::MoveGroup::Plan scan;
  robot_interface_->getFullMotionPlan( *(robot_interface_->current_view_), scan );
  int max_execution_runs = 3;
  int current_run = 0;
    
  if( scan.trajectory_.joint_trajectory.points.size()!=0 )
  {
    ROS_INFO("RemodeDataRetrievalModule: Attempting to execute retrieval trajectory.");
    remode_commander_.publish(START_RECONSTRUCTION);
    remode_has_published_ = false;
    octomap_has_published_=false;
    
    robot_interface_->executeMoveItPlan( scan );
    
    //remode_commander_.publish(STOP_AND_SMOOTH);
    
    ros::Time publish_time = ros::Time::now();
    while( !octomap_has_published_ && !remode_has_published_ && robot_interface_->ros_node_->ok() ) // wait for remode to publish
    {
      if( ros::Time::now() > (publish_time+max_remode_wait_time_) )
      {
	if( current_run>=max_execution_runs )
	{
	  ROS_WARN("No data retrieved in time.");
	  break; // no data received, waited long enough
	}
	else
	{
	  current_run++;
	  robot_interface_->executeMoveItPlan( scan );
	  publish_time = ros::Time::now();
	}
      }
      ros::Duration(0.005).sleep();
      ros::spinOnce();
    }
    
    if( remode_has_published_ )
    {
      return RobotPlanningInterface::RECEIVED;
    }
    else
    {
      return RobotPlanningInterface::RECEPTION_FAILED;
    }
  }
  else
  {
    ROS_WARN("YoubotPlanner::RemodeDataRetriever::retrieveData::Cannot retrieve data because the scanning view path associated with the current view was empty. Retrieving data failed.");
    return RobotPlanningInterface::RECEPTION_FAILED;
  }
}

bool RemodeDataRetriever::movementNeeded()
{
  return true;
}

bool RemodeDataRetriever::getRetrievalMovement( robot_state::RobotState& _state, movements::KinematicMovementDescription* _retrieval_movement, movements::KinematicMovementDescription::PathInfo* _additional_info )
{
  movements::Pose link_4_pose = robot_interface_->linkPoseInRobotState("arm_link_4",_state);
  
  // radius, 4 turns/sec, 0.025 m/s radial speed ->2s to reach limit
  (*_retrieval_movement) = movements::InOutSpiral::create( link_4_pose.orientation, scanning_radius_, 4*6.283185307, 0.025, movements::InOutSpiral::ZXPlane );
  
  _additional_info->start_time = 0.5;
  _additional_info->end_time = 0.9; //before: 3.5
  
  return true;
}

void RemodeDataRetriever::octomapCallback( const octomap_msgs::OctomapConstPtr& _msg )
{
  octomap_has_published_ = true;
}

void RemodeDataRetriever::remodeCallback( const sensor_msgs::PointCloud2::ConstPtr& _remode_cloud )
{
  remode_has_published_ = true;
  robot_interface_->stop_data_retrieval_movement_ = true;
  remode_commander_.publish(SET_IDLE);
}

}