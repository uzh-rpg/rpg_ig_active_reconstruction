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

#include <Eigen/Core>
#include <thread>
#include <mutex>
#include <movements/core>
#include <tf/transform_broadcaster.h>

namespace flying_gazebo_stereo_cam
{
  /*! Simple ROS interface for moving a stereo camera within gazebo. Poses are assumed exactly without added noise.
   * It also features a TF information broadcaster.
   */
  class Controller
  {
  public:
    
    /*! Constructor.
     * @param cam_model_name Name of the spawned model in gazebo. Used to identify it.
     */
    Controller(std::string cam_model_name);
    
    /*! Stops the thread on destruction.*/
    virtual ~Controller();
    
    /*! Tells the robot to get the camera to a new view
    * @param new_pose where to move to
    * @return false if the operation failed
    */
    virtual bool moveTo( movements::Pose new_pose );
    
    /*! Returns the current camera pose.
     * @throws std::runtime_error If reception failed.
     */
    virtual movements::Pose currentPose();
    
    /*! Starts a TF publisher in another thread.
     * @param camera_frame_name Name of the camera coordinate frame to be published.
     * @param world_frame_name Name of the world coordinate frame to be published.
     */
    virtual void startTfPublisher(std::string camera_frame_name, std::string world_frame_name);
    
    /*! Stops publishing tf information. Returns immediately, while the publishing loop only stops at its exit point.
     */
    virtual void stopTfPublisher();
    
  protected:
    /*! Tf publishing thread.
     */
    virtual void keepPublishing(std::string camera_frame_name, std::string world_frame_name);
    
  private:
    std::string cam_model_name_;
    bool has_moved_;
    
    bool keepPublishing_; //! Thread runs as long as this is true
    std::thread publisher_;
    std::mutex protector_;
    
    movements::Pose current_pose_; //! Current pose, undefined until first moveTo call was issued.
    Eigen::Quaterniond cam_to_image_; //! Transformation from camera model to image frame.
    
    tf::TransformBroadcaster tf_broadcaster_; //! Broadcaster for tf
  };
  
}