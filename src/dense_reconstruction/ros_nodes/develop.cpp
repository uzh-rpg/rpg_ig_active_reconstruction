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
#include "boost/foreach.hpp"

#include <random>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "youbot_reconstruction_controller");
  ros::NodeHandle n("youbot_reconstruction_controller");
  
    
  dense_reconstruction::YoubotPlanner calibrator(&n);
  
  std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > joint_values;
  std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > grid;
  calibrator.calculateArmGrid( 60, 60, joint_values, &grid );
  
  std::ofstream out("/home/stewess/Documents/youbot_arm_grid_50pts_per_m.txt", std::ofstream::trunc);
  BOOST_FOREACH( auto point, grid )
  {
    out<<point(0)<<" "<<point(1)<<"\n";
  }
  out.close();
  
  std::ofstream out2("/home/stewess/Documents/youbot_arm_joint_values_50pts_per_m.txt", std::ofstream::trunc);
  BOOST_FOREACH( auto value, joint_values )
  {
    out2<<value(0)<<" "<<value(1)<<" "<<value(2)<<"\n";
  }
  out2.close();
  
  return 0;
  
  ros::Rate rate(0.2);
  
  while ( calibrator.runSingleIteration() && n.ok() )
  {
    cout<<endl<<"Working: "<<endl;
  }
  
  return 0;
} 
