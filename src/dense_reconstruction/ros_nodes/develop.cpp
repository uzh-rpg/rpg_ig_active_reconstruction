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

 
#include "dense_reconstruction/youbot_planning.h"

#include <random>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "youbot_reconstruction_controller");
  ros::NodeHandle n("youbot_reconstruction_controller");
  
    
  dense_reconstruction::YoubotPlanner calibrator(&n);
  
  
  ros::Rate rate(0.2);
  
  while ( calibrator.runSingleIteration() && n.ok() )
  {
    cout<<endl<<"Working: "<<endl;
  }
  
  return 0;
} 
