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


#include "ig_active_reconstruction/world_representation_communication_interface.hpp"


namespace ig_active_reconstruction
{
  
namespace world_representation
{
  
  CommunicationInterface::IgRetrievalCommand::IgRetrievalCommand()
  : ray_resolution_x(1.0)
  , ray_resolution_y(1.0)
  , ray_step_size(1)
  , min_ray_depth(0.0)
  , max_ray_depth(10.0)
  , occupied_passthrough_threshold(0.0)
  {
    ray_window.min_x_perc = 0.0;
    ray_window.max_x_perc = 1.0;
    ray_window.min_y_perc = 0.0;
    ray_window.max_y_perc = 1.0;
  }
  
}


}