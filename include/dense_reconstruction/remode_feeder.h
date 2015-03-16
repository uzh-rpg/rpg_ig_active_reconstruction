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

/** simple node can act as interface to remode, feeding it data
 * The idea was that data from different sources could be merged here (e.g. SVO, Odometry, etc), currently unfinished
 * Not used at the moment.
 */

#pragma once

#include "ros/ros.h"
#include <svo_msgs/DenseInputWithFeatures.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <movements/core>
#include <gazebo_msgs/LinkStates.h>

namespace dense_reconstruction
{

class RemodeFeeder
{
public:
  RemodeFeeder( ros::NodeHandle& _n, unsigned int _publish_every_xth_frame = 1 );
  
  /** image callback function */
  void imageStreamCallback(  const sensor_msgs::ImageConstPtr& _newImage );
  
  /** Loads a transform from TF that transform entities from _source to _target
  * @param _source source frame
  * @param _target target frame
  * @param _time time for which the transform is seeked
  * @param _output output pose
  * @param _max_wait_time maximal time to wait for the transformation to be available on tf tree [s]
  * @return true if successfully found transformation in given time
  */
  bool poseFromTF( std::string _source, std::string _target, ros::Time _time, movements::Pose* _output, double _max_wait_time=3 );
  
  void gazeboCallback( const gazebo_msgs::LinkStatesConstPtr& _gazebo_states );
  
  /** svo callback function */
  void svoCallback( const svo_msgs::DenseInputWithFeaturesConstPtr& _svo_output );
private:
  ros::NodeHandle nh_;
  ros::Publisher feeder_;
  ros::Subscriber image_stream_;
  ros::Subscriber svo_subscriber_;
  
  tf::TransformListener tf_listener_;
  //ros::Subscriber svo_; // not used yet would be used e.g. for features
  
  std::string world_frame_;
  std::string camera_frame_;
  
  double min_depth_; // for remode
  double max_depth_; // for remode
  unsigned int publish_every_xth_frame_; // publish every xth frame
  unsigned int publish_count_;
  
  bool got_gazebo_pose_;
  gazebo_msgs::LinkStates last_gazebo_msg_;
  movements::Pose groundTruthStateFromGazebo();
};

}