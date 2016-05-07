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


#include "ig_active_reconstruction_octomap/octomap_ig_tree_world_representation.hpp"
#include "ig_active_reconstruction_octomap/octomap_ray_occlusion_calculator.hpp"
#include "ig_active_reconstruction_octomap/octomap_std_pcl_input_point_xyz.hpp"
#include "ig_active_reconstruction_octomap/octomap_basic_ray_ig_calculator.hpp"
#include "ig_active_reconstruction_octomap/ig/occlusion_aware.hpp"
#include "ig_active_reconstruction_octomap/ig/unobserved_voxel.hpp"
#include "ig_active_reconstruction_octomap/ig/rear_side_voxel.hpp"
#include "ig_active_reconstruction_octomap/ig/rear_side_entropy.hpp"
#include "ig_active_reconstruction_octomap/ig/proximity_count.hpp"
#include "ig_active_reconstruction_octomap/ig/vasquez_gomez_area_factor.hpp"
#include "ig_active_reconstruction_octomap/ig/average_entropy.hpp"

#include "ig_active_reconstruction_ros/param_loader.hpp"
#include "ig_active_reconstruction_ros/world_representation_ros_server_ci.hpp"

/*! Implements a ROS node holding an octomap world represenation and listening on a PCL topic.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "octomap_world_representation");
  ros::NodeHandle nh;
  
  namespace iar = ig_active_reconstruction;
  
  using namespace iar::world_representation::octomap;
  typedef IgTreeWorldRepresentation WorldRepresentation;
  typedef WorldRepresentation::TreeType TreeType;
  typedef StdPclInputPointXYZ<TreeType>::PclType PclType;
  
  
  // Load parameters
  // .............................................................................................
  // Octree config
  TreeType::Config octree_config;
  ros_tools::getParam(octree_config.resolution_m,"resolution_m",0.01);
  ros_tools::getParam(octree_config.occupancy_threshold,"occupancy_threshold",0.5);
  ros_tools::getParam(octree_config.hit_probability,"hit_probability",0.7);
  ros_tools::getParam(octree_config.miss_probability,"miss_probability",0.4);
  ros_tools::getParam(octree_config.clamping_threshold_min,"clamping_threshold_min",0.12);
  ros_tools::getParam(octree_config.clamping_threshold_max,"clamping_threshold_max",0.97);
  
  // Input config
  StdPclInputPointXYZ<TreeType>::Type::Config input_config;
  ros_tools::getParam(input_config.use_bounding_box,"use_bounding_box",false);
  ros_tools::getParamIfAvailable<float,double>(input_config.bounding_box_min_point_m.x(),"bounding_box_min_point_m/x");
  ros_tools::getParamIfAvailable<float,double>(input_config.bounding_box_min_point_m.y(),"bounding_box_min_point_m/y");
  ros_tools::getParamIfAvailable<float,double>(input_config.bounding_box_min_point_m.z(),"bounding_box_min_point_m/z");
  ros_tools::getParamIfAvailable<float,double>(input_config.bounding_box_max_point_m.x(),"bounding_box_max_point_m/x");
  ros_tools::getParamIfAvailable<float,double>(input_config.bounding_box_max_point_m.y(),"bounding_box_max_point_m/y");
  ros_tools::getParamIfAvailable<float,double>(input_config.bounding_box_max_point_m.z(),"bounding_box_max_point_m/z");
  ros_tools::getParam(input_config.max_sensor_range_m,"max_sensor_range_m",-1.0);
  
  // Occlusion calculation config
  RayOcclusionCalculator<TreeType,PclType>::Options occlusion_config(0.3);
  ros_tools::getParam(occlusion_config.occlusion_update_dist_m,"occlusion_update_dist_m",0.3);
  
  
  
  
  
  // Instantiate main world object
  // .............................................................................................
  WorldRepresentation world_representation(octree_config);
  
  // Add input
  // .............................................................................................
  typename StdPclInputPointXYZ<TreeType>::Ptr std_input = world_representation.getLinkedObj<StdPclInputPointXYZ>(input_config);
  // calculate occlusion
  std_input->setOcclusionCalculator<RayOcclusionCalculator>(occlusion_config);
  
  // Add information gain calculator
  // .............................................................................................
  BasicRayIgCalculator<IgTreeWorldRepresentation::TreeType>::Ptr ig_calculator = world_representation.getLinkedObj<BasicRayIgCalculator>();
  
  // set information gains that shall be used
  ig_calculator->registerInformationGain<OcclusionAwareIg>();
  ig_calculator->registerInformationGain<UnobservedVoxelIg>();
  ig_calculator->registerInformationGain<RearSideVoxelIg>();
  ig_calculator->registerInformationGain<RearSideEntropyIg>();
  ig_calculator->registerInformationGain<ProximityCountIg>();
  ig_calculator->registerInformationGain<VasquezGomezAreaFactorIg>();
  ig_calculator->registerInformationGain<AverageEntropyIg>();
  
  // Expose the information gain calculator to ROS
  
  
  return 0;
}