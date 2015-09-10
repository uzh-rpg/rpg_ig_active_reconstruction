#include "dense_reconstruction/model_completeness_calculator.h"
#include <pcl/kdtree/kdtree_flann.h>

namespace dense_reconstruction
{
  
  void ModelCompletenessCalculator::setInputBag( std::string path )
  {
    bag_.open( path, rosbag::bagmode::Read );
    
    std::vector<std::string> topics;
    topics.push_back("remode/pointcloud_single");
    rosbag::View viewer( bag_, rosbag::TopicQuery(topics) );
    
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::PointCloud<pcl::PointXYZ> icpFilteredNew;
    
    BOOST_FOREACH( rosbag::MessageInstance const m, viewer )
    {
      sensor_msgs::PointCloud2::Ptr msg = m.instantiate<sensor_msgs::PointCloud2>();
      bagContent_.push_back(msg);
    }
    
    for( unsigned int i=0; i<bagContent_.size(); ++i )
    {
      pcl::fromROSMsg(*bagContent_[i], pc);
      if(i=0)
      {
	cloud += pc;
      }
      else
      {
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource( pc.makeShared() );
        icp.setInputTarget( cloud.makeShared() );
        icp.setMaxCorrespondenceDistance( 0.0005 );
	icp.align( icpFilteredNew );
	
	if( icp.hasConverged() )
	{
	  cloud+=icpFilteredNew;
	}
	else
	{
	  icpFilteredNew=pc;
	  cloud+=pc;
	}
      }
      
      completeness.push_back( calculateCompleteness(cloud) );
      std::string path = path_+"/pclStep"+std::to_string(i)+".ply";
      pcl::io::savePLYFile(path,icpFilteredNew,false);
      std::string total = path_+"/pclIntegrated"+std::to_string(i)+".ply";
      pcl::io::savePLYFile(total,cloud);
      
      
    }
    
  }
  
  void ModelCompletenessCalculator::setGroundTruth( std::string path )
  {
    
  }
  
  void ModelCompletenessCalculator::calculateCompleteness( std::vector<double>& completeness )
  {
    
  }
  
  double ModelCompletenessCalculator::calculateCompleteness( pcl::PointCloud<pcl::PointXYZ>& toCompare )
  {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud( toCompare.makeShared() );
    
    unsigned int pointsInGroundTruth = groundTruth_.points.size();
    unsigned int registeredPoints = 0;
    
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = groundTruth_.begin(); it != groundTruth_.end(); ++it )
    {
      pcl::PointXYZ searchPoint = *it;
      //point3d point(it->x, it->y, it->z);
      
      if( kdtree.radiusSearch(searchPoint, registerDistance_, pointIdxRadiusSearch, pointRadiusSquaredDistance )>0 )
      {
	++registeredPoints;
      }
    }
    
    return (double)registeredPoints/(double)pointsInGroundTruth;
  }
  
}