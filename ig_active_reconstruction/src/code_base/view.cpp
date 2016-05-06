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

#include "ig_active_reconstruction/view.hpp"

#include <limits>
#include <iostream>

#include "ig_active_reconstruction/octomap_ig_tree_world_representation.hpp"
#include "ig_active_reconstruction/octomap_ray_occlusion_calculator.hpp"
#include "ig_active_reconstruction/octomap_std_pcl_input.hpp"
#include "ig_active_reconstruction/octomap_basic_ray_ig_calculator.hpp"
#include "ig_active_reconstruction/ig/occlusion_aware.hpp"
#include "ig_active_reconstruction/ig/unobserved_voxel.hpp"
#include "ig_active_reconstruction/ig/rear_side_voxel.hpp"
#include "ig_active_reconstruction/ig/rear_side_entropy.hpp"
#include "ig_active_reconstruction/ig/proximity_count.hpp"
#include "ig_active_reconstruction/ig/vasquez_gomez_area_factor.hpp"
#include "ig_active_reconstruction/ig/average_entropy.hpp"
#include "ig_active_reconstruction/views_simple_view_space_module.hpp"

namespace ig_active_reconstruction
{
  
namespace views
{

View::IdType View::runningIndex_ = 0;

View::View():
  index_(runningIndex_++),
  is_reachable_(true),
  is_bad_(false),
  visited_(0)
{
  if( runningIndex_==std::numeric_limits<IdType>::max() )
    std::cerr<<"Attention::View::index_ is about to overflow! (Next: "<<runningIndex_<<", and the one after: "<<runningIndex_+1<<".";
  
  using namespace world_representation::octomap;
  
  IgTreeWorldRepresentation tree;
  
  auto std_input = tree.getLinkedObj<StdPclInput,pcl::PointCloud<pcl::PointXYZ> >();
  std_input->setOcclusionCalculator<RayOcclusionCalculator>(0.3);
  
  auto ig_calculator = tree.getLinkedObj<BasicRayIgCalculator>();
  ig_calculator->registerInformationGain<OcclusionAwareIg>();
  ig_calculator->registerInformationGain<UnobservedVoxelIg>();
  ig_calculator->registerInformationGain<RearSideVoxelIg>();
  ig_calculator->registerInformationGain<RearSideEntropyIg>();
  ig_calculator->registerInformationGain<ProximityCountIg>();
  ig_calculator->registerInformationGain<VasquezGomezAreaFactorIg>();
  ig_calculator->registerInformationGain<AverageEntropyIg>();
  
  SimpleViewSpaceModule viewspace("blah");
  viewspace.loadFromFile("thetest");
  viewspace.saveToFile("hahaha");
}

View::View( std::string source_frame )
  : index_(runningIndex_++)
  , source_frame_(source_frame)
  , is_reachable_(true)
  , is_bad_(false)
  , non_viewspace_(false)
  , visited_(0)
{
  if( runningIndex_==std::numeric_limits<IdType>::max() )
    std::cerr<<"Attention::View::index_ is about to overflow! (Next: "<<runningIndex_<<", and the one after: "<<runningIndex_+1<<".";
}

View::View( IdType id )
  : index_(id)
  , is_reachable_(true)
  , is_bad_(false)
  , non_viewspace_(false)
  , visited_(0)
{
  
}

movements::Pose& View::pose()
{
  return pose_;
}

std::string& View::sourceFrame()
{
  return source_frame_;
}

bool& View::reachable()
{
  return is_reachable_;
}

unsigned int& View::timesVisited()
{
  return visited_;
}

bool& View::bad()
{
  return is_bad_;
}

bool& View::nonViewSpace()
{
  return non_viewspace_;
}

View::IdType View::index() const
{
  return index_;
}

std::vector<std::string>& View::additionalFieldsNames()
{
  return additional_fields_names_;
}

std::vector<double>& View::additionalFieldsValues()
{
  return additional_fields_values_;
}

boost::shared_ptr<ig_active_reconstruction::views::View::ViewInfo>& View::associatedData()
{
  return associated_data_;
}

}

}

std::ostream& operator<<(std::ostream& _out, ig_active_reconstruction::views::View& view )
{
  _out<<"Pose:\n";
  _out<<"  position: \n";
  _out<<"    x: "<<view.pose().position.x()<<"\n";
  _out<<"    y: "<<view.pose().position.y()<<"\n";
  _out<<"    z: "<<view.pose().position.z()<<"\n";
  _out<<"  orientation: \n";
  _out<<"    x: "<<view.pose().orientation.x()<<"\n";
  _out<<"    y: "<<view.pose().orientation.y()<<"\n";
  _out<<"    z: "<<view.pose().orientation.z()<<"\n";
  _out<<"    w: "<<view.pose().orientation.w()<<"\n";
  return _out;
}