#include "dense_reconstruction/rosbag_creator.h"

RosbagCreator::RosbagCreator( std::string bagName )
: filename_(bagName)
{
    pclTopic_ = "/remode/pointcloud_single";
    octOccTopic_ = "/occupied_cells_vis_array";
    octFreeTopic_ = "/free_cells_vis_array";
    tfTopic_ = "/tf";
    octFullTopic_ = "/octomap_full";
    pclOctoTopic_ = "/dense_reconstruction/model/pointcloud";
    
    start();
}

void RosbagCreator::start()
{
    bag_.open( filename_, rosbag::bagmode::Write );
    
    ros::NodeHandle n;
    pclCbSub_ = n.subscribe(pclTopic_, 10, &RosbagCreator::pclCallback, this);
    octOccCbSub_ = n.subscribe(octOccTopic_, 10, &RosbagCreator::octomapOccupiedCb, this);
    tfCbSub_ = n.subscribe(tfTopic_, 10, &RosbagCreator::tfCallback, this);
    octFuCbSub_ = n.subscribe(octFullTopic_, 10, &RosbagCreator::octomapFullCb, this);
    octFrCbSub_ = n.subscribe(octFreeTopic_, 10, &RosbagCreator::octomapFreeCb, this);
    //pclOctoTopicCbSub_ = n.subscribe(pclOctoTopic_, 10, &RosbagCreator::pclOctoTopicCb, this);
}

void RosbagCreator::stop()
{
    pclCbSub_.shutdown();
    octOccCbSub_.shutdown();
    tfCbSub_.shutdown();
    octFuCbSub_.shutdown();
    octFrCbSub_.shutdown();
    
    bag_.close();
}

void RosbagCreator::pclCallback( const sensor_msgs::PointCloud2ConstPtr& _msg )
{
    bag_.write(pclTopic_, ros::Time::now(), _msg );
}

void RosbagCreator::octomapOccupiedCb( const visualization_msgs::MarkerArrayConstPtr& _msg )
{
    bag_.write(octOccTopic_, ros::Time::now(), _msg );
}

void RosbagCreator::tfCallback( const tf2_msgs::TFMessageConstPtr& _msg )
{
    bag_.write(tfTopic_, ros::Time::now(), _msg );
}

void RosbagCreator::octomapFullCb( const octomap_msgs::OctomapConstPtr& _msg )
{
    bag_.write(octFullTopic_, ros::Time::now(), _msg );
}

void RosbagCreator::octomapFreeCb( const visualization_msgs::MarkerArrayConstPtr& _msg )
{
    bag_.write(octFreeTopic_, ros::Time::now(), _msg );
}

void RosbagCreator::pclOctoTopicCb( const sensor_msgs::PointCloud2ConstPtr& _msg )
{
    bag_.write(pclOctoTopic_, ros::Time::now(), _msg );
}