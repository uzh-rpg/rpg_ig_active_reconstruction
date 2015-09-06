/*! Utility class to save several topics to a rosbag.
 */
#pragma once

#include <iostream>
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_msgs/TFMessage.h>
#include <octomap_msgs/Octomap.h>

class RosbagCreator
{
public:
    /*! Constructor
     * \param bagName name and path of the created bagfile
     */
    RosbagCreator( std::string bagName );
    
    void start();
    void stop();
    
    void pclCallback( const sensor_msgs::PointCloud2ConstPtr& _msg );
    void octomapOccupiedCb( const visualization_msgs::MarkerArrayConstPtr& _msg );
    void tfCallback( const tf2_msgs::TFMessageConstPtr& _msg );
    void octomapFullCb( const octomap_msgs::OctomapConstPtr& _msg );
    void octomapFreeCb( const visualization_msgs::MarkerArrayConstPtr& _msg );
    void pclOctoTopicCb( const sensor_msgs::PointCloud2ConstPtr& _msg );
    
private:
    rosbag::Bag bag_;
    std::string filename_;
    
    std::string pclTopic_;
    std::string octOccTopic_;
    std::string octFreeTopic_;
    std::string tfTopic_;
    std::string octFullTopic_;
    std::string pclOctoTopic_;
    
    ros::Subscriber pclCbSub_;
    ros::Subscriber octOccCbSub_;
    ros::Subscriber tfCbSub_;
    ros::Subscriber octFuCbSub_;
    ros::Subscriber octFrCbSub_;
    ros::Subscriber pclOctoTopicCbSub_;
};