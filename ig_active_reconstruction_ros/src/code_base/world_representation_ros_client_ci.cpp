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

#include <stdexcept>

#include "ig_active_reconstruction_ros/world_representation_ros_client_ci.hpp"
#include "ig_active_reconstruction_ros/conversions.hpp"

#include "ig_active_reconstruction_msgs/InformationGainCalculation.h"
#include "ig_active_reconstruction_msgs/MapMetricCalculation.h"
#include "ig_active_reconstruction_msgs/StringList.h"


namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
  RosClientCI::RosClientCI( ros::NodeHandle nh )
  : nh_(nh)
  {
    view_ig_computation_ = nh.serviceClient<ig_active_reconstruction_msgs::InformationGainCalculation>("world/information_gain");
    map_metric_computation_ = nh.serviceClient<ig_active_reconstruction_msgs::MapMetricCalculation>("world/map_metric");
    available_ig_receiver_ = nh.serviceClient<ig_active_reconstruction_msgs::StringList>("world/ig_list");
    availalbe_mm_receiver_ = nh.serviceClient<ig_active_reconstruction_msgs::StringList>("world/mm_list");
  }
  
}

}