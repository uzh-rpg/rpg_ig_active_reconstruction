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

#pragma once


#include "ros/ros.h"
#include <movements/core>
#include <movements/ros_movements.h>

#include "dense_reconstruction/robot_planning_interface.h"
#include "dense_reconstruction/ViewInformationReturn.h"
#include "std_msgs/String.h"
#include "dense_reconstruction/SaveViewSpace.h"


namespace dense_reconstruction
{

/** simple, dynamic programming based next best view planner for dense reconstruction
 */
class ViewPlanner
{
public:
  ViewPlanner( ros::NodeHandle& _n );
  
  /**
   * starts the view planner, it will however only start taking actions when it gets the command on the "dense_reconstruction/view_planner/command" topic
   */
  void run();
  
  /**
   * waits and spins once
   */
  void waitAndSpin( double _sec=0.5 );
  
  /**
   * will pause if requested
   */
  void pauseIfRequested();
  
  /**
   * to calculate a subset, e.g. to limit the range in which the next view is sought, so far it only filters out already visited or bad positions
   */
  void determineAvailableViewSpace( std::vector<unsigned int>& _output );
  
  /**
   * return function calculates the return associated with a view, given the cost and a set of information gains
   */
  double calculateReturn( double _cost, std::vector<double>& _informations );
  
  /**
   * returns true if the values of the next best view fulfill the termination criteria, which
   * means that the computation will be stopped and the result accepted
   * Currently the number of occupied voxels is evaluated and iteration stops if it doesn't change sufficiently anymore.
   */
  bool terminationCriteriaFulfilled( double _return_value, 
                                     double _cost, 
                                     std::vector<double>& _information_gain, 
                                     std::vector<double>& _model_statistics );
  
  struct ReturnValueInformation
  {
    double return_value;
    double winning_margin;
    double return_value_mean;
    double return_value_stddev;
  };
  
  /**
   * stores nbv data in the planning_data_ structures which are printed to file when the
   * program finishes.
   * @param _nbv_index index of the nbv - needed to get the pose associated to it
   * @param _return_value_information information struct regarding the return value of the NBV
   * @param _cost movement cost to reach the view
   * @param _information_gain information gains calculated for the NBV
   * @param _model_statistics overall model statistics
   * @param _additional_field_names (optional) pointer to vector with the names of the additional fields
   * @param _additional_field_values (optional) values for the additional fields, if its size doesn't match the size of _additional_field_names, then the information is dropped
   */
  void saveNBVData( unsigned int _nbv_index, 
                    ReturnValueInformation& _return_value_information, 
                    double _cost, 
                    std::vector<double>& _information_gain, 
                    std::vector<double>& _model_statistics, 
                    std::vector<std::string>* _additional_field_names=nullptr, 
                    std::vector<double>* _additional_field_values=nullptr );
  
  /**
   * saves the data stored in planning_data_ to a file in the data_folder_ folder.
   * File format is:
   * name1 name2 ... nameX
   * data1_t1 data2_t1 ... dataX_t1
   * data1_t2 data2_t2 ... dataX_t2
   * ...
   */
  void saveDataToFile();
  
  /**
   * stores all the data that was retrieved to choose the next best view
   * @param _views_to_consider view space index associated with each array position
   * @param _view_returns information struct regarding the return value of the NBV
   * @param _costs movement cost to reach the view (summed as given by robot)
   * @param _information_gain information gains calculated for the NBV
   * @param _model_statistics overall model statistics
   * @param _detailed_costs cost fields (parts of which the robot movement cost consisted of)
   */
  void saveCurrentChoiceDataToFile( std::vector<unsigned int> _views_to_consider, 
                                    std::vector<double> _view_returns, 
                                    std::vector<double> _costs, 
                                    std::vector< std::vector<double> >& _information_gain, 
                                    std::vector<double>& _model_statistics, 
                                    std::vector<RobotPlanningInterface::MovementCost>* _detailed_costs=nullptr );
  
  /**
   * calls the data retrieval service and waits until it succeeded
   * Can be aborted and paused through commands
   */
  void retrieveDataAndWait( double _sec=0.5 );
  
  /**
   * calls the robots movement service
   * Can be aborted and paused through commands (stop will while called in this loop only stop the loop, not the whole program)
   */
  bool moveToAndWait( View& _target_view, double _sec=0.5 );
  
  /**
   * returns the index of the field with name _name in the planning_data_ structure, creates a
   * new one if none exists yet
   */
  unsigned int getIndexForAdditionalField( std::string _name );
  
  /** attempts to load the view space from service
   * returns true if successful
   */
  bool getViewSpace();
  
  /**
   * retrieves the current view by calling the service
   */
  bool getCurrentView( View& _output);
  
  /**
   * retrieves data by calling the surface
   */
  bool retrieveData( RobotPlanningInterface::ReceiveInfo& _output );
  
  /**
   * gets the movement cost for given start- and endpoints
   * returns true if calling the service was successful
   */
  bool movementCost( RobotPlanningInterface::MovementCost& _output, View& _start_view, View& _target_view );
  
  /**
   * calls the service to move the robot somewhere
   * Returns true if calling the service was successful
   */
  bool moveTo( bool& _output, View& _target_view );
  
  /**
   * retreives expected informations for given poses (if more than one then it's a path into the future)
   * @param _output the calculated information  values for the given metrics
   * @param _poses the last pose is the one for which the information is sought, optional other poses describe a path into the future before the sought view information
   * @param _metric_names set of metrics to use to calculate the information
   * @return true if the view information service was called successfully
   */
  bool getViewInformation( std::vector<double>& _output, movements::PoseVector& _poses );
  
  /** Function for threaded view information calls
   * @param igVector Vector that is to be filled with the information gains.
   * @param viewSet Set of corresponding view for which the ig's are to be calculated.
   * @param cost Vector with costs.
   * @param slotId Which slot shall be filled in this function: E.g.: slotId=2, nrOfSlots=5 -> function calculates IG's for views 2,7,12,...
   * @param nrOfSlots How many slots there are in total
   */
  void threadedIGCalculation(  std::vector< std::vector<double> >& igVector, 
			       std::vector<unsigned int>& viewSet, 
			       std::vector<double>& cost,
			       unsigned int slotId, 
			       unsigned int nrOfSlots );
  
  /**
   * retreives expected informations for the complete current model state
   * @param _output the calculated statistic values for the given metrics
   * @return true if the view information service was called successfully
   */
  bool getModelInformation( std::vector<double>& _output );
  
  void commandCallback( const std_msgs::StringConstPtr& _msg );
  
  bool saveViewSpaceToFileService( SaveViewSpace::Request& _req, SaveViewSpace::Response& _res );
private:
    unsigned int occupiedMetricId_;
    unsigned int previousNrOfOccupieds_;
    double saturationThreshold_; // If the occupancy nr doesn't change more than this anymore, end the iteration
    
    bool onlyIterateViewSpace_;
    unsigned int occplaneMetricId_;
  std::string planning_frame_;
  std::vector<std::string> metrics_to_use_; // metrics used to evalute views
  std::vector<std::string> model_metrics_; // metrics used to evaluate the current model
  
  std::string experiment_id_; // string describing the current run, saved as part of file names
  
  ros::ServiceServer save_view_space_server_;
  
  ViewSpace view_space_;
  
  View current_view_;
  
  unsigned int max_vp_visits_; // how many times a view point may be visited at max
  
  bool start_;
  bool pause_;
  bool stop_and_print_;
  bool reinit_;
  bool abort_loop_;
  
  bool random_nbv_;
  
  bool observe_timing_;
  std::vector< std::vector<double> > timing_;
  
  double cost_weight_;
  double information_weight_;
  std::vector<double> information_weights_;
  
  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_int_distribution<unsigned int> distr_;
  
  bool mark_visited_; /// if set to true then views marked as already visited won't be taken into account for the next best view calculations
  
  double ray_resolution_x_;
  double ray_resolution_y_;
  double ray_step_size_;
  double subwindow_width_percentage_;
  double subwindow_height_percentage_;
  double min_ray_depth_;
  double max_ray_depth_;
  double occupied_passthrough_threshold_;
  
  unsigned int max_movement_attempts_; /// max nr of times a movement is attempted for a given view until it is rejected and flagged as "bad"
  
  std::vector< std::vector<double> > planning_data_; /// container for data gathered during planning: visited views, informations and costs
  std::vector< std::string > planning_data_names_; /// names of the data stored in planning_data_
  std::string data_folder_;
  
  ros::NodeHandle nh_;
  ros::Subscriber command_;
  
  ros::ServiceClient view_space_retriever_;
  ros::ServiceClient current_view_retriever_;
  ros::ServiceClient data_retriever_;
  ros::ServiceClient cost_retriever_;
  ros::ServiceClient view_information_retriever_;
  ros::ServiceClient robot_mover_;
};

}