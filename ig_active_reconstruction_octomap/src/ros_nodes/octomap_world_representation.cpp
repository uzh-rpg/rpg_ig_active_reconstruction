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

/*! Implements a ROS node holding an octomap world represenation and listening on a PCL topic.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "octomap_world_representation");
  ros::NodeHandle nh;
  
  namespace iar = ig_active_reconstruction;
  
  using namespace iar::world_representation::octomap;
  
  IgTreeWorldRepresentation tree;
  typedef IgTreeWorldRepresentation::TreeType TreeType;
  typedef StdPclInputPointXYZ<TreeType>::PclType PclType;
  
  typename StdPclInputPointXYZ<TreeType>::Ptr std_input = tree.getLinkedObj<StdPclInputPointXYZ>();
  std_input->setOcclusionCalculator<RayOcclusionCalculator>(RayOcclusionCalculator<TreeType,PclType>::Options(0.3));
  
  BasicRayIgCalculator<IgTreeWorldRepresentation::TreeType>::Ptr ig_calculator = tree.getLinkedObj<BasicRayIgCalculator>();
  ig_calculator->registerInformationGain<OcclusionAwareIg>();
  ig_calculator->registerInformationGain<UnobservedVoxelIg>();
  ig_calculator->registerInformationGain<RearSideVoxelIg>();
  ig_calculator->registerInformationGain<RearSideEntropyIg>();
  ig_calculator->registerInformationGain<ProximityCountIg>();
  ig_calculator->registerInformationGain<VasquezGomezAreaFactorIg>();
  ig_calculator->registerInformationGain<AverageEntropyIg>();
  
  return 0;
}