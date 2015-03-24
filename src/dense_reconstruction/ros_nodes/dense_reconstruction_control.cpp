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

// control node for active dense reconstruction that allows the user to interact with the process through services

#include <iostream>
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

using namespace std;

void printCommands()
{
  cout<<endl<<endl<<"DENSE RECONSTRUCTION CONTROL INTERFACE"<<endl<<"----------------------------------";
  cout<<endl<<"The following commands are available:";
  cout<<endl<<"g: start planning, unpause";
  cout<<endl<<"b: break, pause planning";
  cout<<endl<<"s: stop planning and save data";
  cout<<endl<<"a: abort loop (not everywhere supported)";
  cout<<endl<<"p: print data, without stopping";
  cout<<endl<<"i: initialize/reinitialize tf";
  cout<<endl<<"q: quit program";
  cout<<endl<<endl;
}

void sendPlannerCommand( std::string _command, ros::Publisher& _publisher )
{
  std_msgs::String command;
  command.data = _command;
  _publisher.publish(command);
}

void reinitializeTFCommand()
{
  std_srvs::Empty service;
  ros::service::call("/dense_reconstruction/robot_interface/setup_tf",service);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dense_reconstruction_control");
  ros::NodeHandle n;
  
  ros::Publisher planner_commands = n.advertise<std_msgs::String>("/dense_reconstruction/view_planner/command",1);
  
  
  ROS_INFO("Starting dense_reconstruction_control");
  
  char userInput;
  
  while( n.ok() )
  {
    printCommands();
    cout<<endl<<"Enter command."<<endl;
    cin >> userInput;
    switch(userInput)
    {
      case 'g':
	sendPlannerCommand( "START", planner_commands );
	break;
      case 'b':
	sendPlannerCommand( "PAUSE", planner_commands );
	break;
      case 's':
	sendPlannerCommand( "STOP_AND_PRINT", planner_commands );
	break;
      case 'a':
	sendPlannerCommand( "ABORT_LOOP", planner_commands );
	break;
      case 'p':
	sendPlannerCommand( "PRINT_DATA", planner_commands );
	break;
      case 'i':
	reinitializeTFCommand();
	break;
      case 'q':
	ros::shutdown();
	break;
    }
    cin.ignore(cin.gcount());
  }
  
  return 0;
} 
