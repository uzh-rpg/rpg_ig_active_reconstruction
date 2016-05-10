/* Copyright (c) 2016, Stefan Isler, islerstefan@bluewin.ch
 * (ETH Zurich / Robotics and Perception Group, University of Zurich, Switzerland)
 *
 * This file is part of ig_active_reconstruction, software for information gain based, active reconstruction.
 *
 * ig_active_reconstruction is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * ig_active_reconstruction is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * Please refer to the GNU Lesser General Public License for details on the license,
 * on <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <thread>
#include <mutex>

#include "ig_active_reconstruction/robot_communication_interface.hpp"
#include "ig_active_reconstruction/views_communication_interface.hpp"
#include "ig_active_reconstruction/world_representation_communication_interface.hpp"
#include "ig_active_reconstruction/utility_calculator.hpp"
#include "ig_active_reconstruction/goal_evaluation_module.hpp"
#include "ig_active_reconstruction/view_space.hpp"

namespace ig_active_reconstruction
{
  
  /*! Basic implementation of a view planner with a static viewspace loaded at the beginning.
   * 
   * Iterates through the following steps:
   * - Issue data collection command
   * - First iteration only: Demand view space, otherwise calculate viewspace subset based on settings.
   * - Get cost for each view candidate
   * - Get predicted information gain for each view candidate
   * - Calculate utility function.
   * - Choose next best view with highest utility function result.
   * - Check if termination criterion is fulfilled.
   * - Move to next best view
   */
  class BasicViewPlanner
  {
  public:
    enum struct Status
    {
      UNINITIALIZED, // Still waiting for necessary data, can't be run yet
      IDLE, // Not running, ready to be (re-)started.
      PAUSED,
      DEMANDING_NEW_DATA,
      DEMANDING_VIEWSPACE,
      NBV_CALCULATIONS, // includes cost and ig retrieval
      DEMANDING_MOVE
    };
    
    struct Config
    {
    public:
      /*! Constructor sets default values.
       */
      Config();
      
    public:
      bool discard_visited; //! Whether views should be discarded once visited. Default: false.
      int max_visits; //! Maximal number a view can be visited before it is discarded, -1 = infinite. Default: -1.
    };
    
  public:
    /*! Constructor.
     */
    BasicViewPlanner( Config config = Config() );
    
    virtual ~BasicViewPlanner();
    
    /*! Sets the robot communication interface with which the view planner corresponds.
     * Can't be set if running.
     */
    virtual void setRobotCommUnit( boost::shared_ptr<robot::CommunicationInterface> robot_comm_unit );
    
    /*! Sets the viewspace communication interface with which the view planner corresponds.
     * Can't be set if running.
     */
    virtual void setViewsCommUnit( boost::shared_ptr<views::CommunicationInterface> views_comm_unit );
    
    /*! Sets the world representation communication interface with which the view planner corresponds.
     * Can't be set if running.
     */
    virtual void setWorldCommUnit( boost::shared_ptr<world_representation::CommunicationInterface> world_comm_unit );
    
    
    /*! Sets the utility function calculator that is to be used. It defines which and how information gain metrics are used.
     * Can't be set if running.
     */
    virtual void setUtility( boost::shared_ptr<UtilityCalculator> utility_calculator );
    
    /*! Sets a goal evaluation module which determines if the view planner shall continue or not.
     * Can't be set if running.
     */
    virtual void setGoalEvaluationModule( boost::shared_ptr<GoalEvaluationModule> goal_evaluation_module );
    
    /*! Starts the procedure in its own thread if it was stopped, continues the procedure if it was paused.
     * @return True if the procedure started successfully, false if not (e.g. because no all necessary parameters are set, like the communication units)
     */
    virtual bool run();
    
    /*! Pauses the procedure if it is running, does nothing otherwise.
     * The call returns immediately, even though the procedure continues until it reaches a breakpoint.
     */
    virtual void pause();
    
    /*! Stops the procedure if it is running, does nothing otherwise.
     * The call returns immediately, even though the procedure continues until it reaches an exit point.
     */
    virtual void stop();
    
    /*! Returns the current procedure status.
     */
    virtual Status status();
    
  protected:
    /*! Returns if the view planner is ready to rumble.
     */
    bool isReady();
    
    /*! Main routine.
     */
    void main();
    
    /*! Pausing if set. TODO: use condition variable
     */
    void pausePoint();
    
  protected:
    Config config_; //! View planner configuration.
    
    boost::shared_ptr<robot::CommunicationInterface> robot_comm_unit_; //! Robot communication interface with which the view planner corresponds.
    boost::shared_ptr<views::CommunicationInterface> views_comm_unit_; //! Viewspace communication interface with which the view planner corresponds.
    boost::shared_ptr<world_representation::CommunicationInterface> world_comm_unit_; //! World representation communication interface with which the view planner corresponds.
    
    boost::shared_ptr<UtilityCalculator> utility_calculator_; //! Utility calculator for evaluating different views. It also defines which information gains are used.
    boost::shared_ptr<GoalEvaluationModule> goal_evaluation_module_; //! Goal evaluation module which determines if the view planner shall continue or not.
    
    Status status_; //! Current status.
    std::thread running_procedure_; //! Thread for the procedure.
    std::mutex mutex_; //! Data guard.
    bool runProcedure_; //! True as long as the procedure is running or paused.
    bool pauseProcedure_; //! True if the procedure should pause.
    
    boost::shared_ptr<views::ViewSpace> viewspace_; //! Current viewspace.
    
  };
  
}