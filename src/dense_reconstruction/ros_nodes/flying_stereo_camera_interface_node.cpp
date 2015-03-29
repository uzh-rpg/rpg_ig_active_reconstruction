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

 
#include "dense_reconstruction/flying_stereo_camera_interface.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flying_stereo_camera_interface");
  ros::NodeHandle n;
  
  // YoubotPlanner tests
  dense_reconstruction::FlyingStereoCameraInterface camera(&n);
  
  ros::spin();
  
  ros::shutdown();
  return 0;
  
} 
