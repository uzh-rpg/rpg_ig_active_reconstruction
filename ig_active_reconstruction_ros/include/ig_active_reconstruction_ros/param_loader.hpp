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

#include "ros/ros.h"

namespace ros_tools
{
  /*! Template function that throws a ROS fatal error if the parameter was not provided on the server.
   * 
   * Function is overloaded for one or two template parameters: the version with two provides the following functionality: If the target value is of a type that
   * is not supported by ROS' getParam functions but can be (static_-)casted from another type that is supported, give the local ros-compliant
   * type as second template argument. Don't forget to check in the output whether the result of the typecast is as expected. If the 
   * type is ros-compliant, no template argument needs to be given as it can be deduced.
   * 
   * @param output (output) Where the parameter will be written
   * @param path Path that will be read from the nodehandle
   * @param nodeHandle Node handle (defaults to a private ROS node handle: ros::NodeHandle("~") )
   */
  template<class PARAM_TYPE>
  void getExpParam( PARAM_TYPE& output, std::string path, ros::NodeHandle nodeHandle = ros::NodeHandle("~") )
  {
    if( !nodeHandle.getParam(path,output) )
    {
      ROS_FATAL_STREAM("Expected parameter '"<<path<<"' was not provided on parameter server. Exiting...");
      ros::shutdown();
    }
  }
  template<class PARAM_TYPE, class LOCAL_ROS_COMPLIANT_TYPE>
  void getExpParam( PARAM_TYPE& output, std::string path, ros::NodeHandle nodeHandle = ros::NodeHandle("~") )
  {
    LOCAL_ROS_COMPLIANT_TYPE local;
    if( !nodeHandle.getParam(path,local) )
    {
      ROS_FATAL_STREAM("Expected parameter '"<<path<<"' was not provided on parameter server. Exiting...");
      ros::shutdown();
    }
    else
    {
      output = static_cast<PARAM_TYPE>(local);
      //ROS_INFO_STREAM("Loaded parameter '"<<path<<"':"<<output);
    }
  }
  
  /*! Silent template function that loads a parameter or its default value.
   * 
   * Function is overloaded for one or two template parameters: the version with two provides the following functionality: If the target value is of a type that
   * is not supported by ROS' getParam functions but can be (static_-)casted from another type that is supported, give the local ros-compliant
   * type as second template argument. Don't forget to check in the output whether the result of the typecast is as expected. If the 
   * type is ros-compliant, no template argument needs to be given as it can be deduced.
   * 
   * @param output (output) Where the parameter will be written
   * @param path Path that will be read from the nodehandle
   * @param defaultValue Default value that will be loaded if not provided
   * @param nodeHandle Node handle (defaults to a private ROS node handle: ros::NodeHandle("~") )
   */
  template<class PARAM_TYPE, class LOCAL_ROS_COMPLIANT_TYPE>
  void getParamSilent( PARAM_TYPE& output, std::string path, PARAM_TYPE defaultValue, ros::NodeHandle nodeHandle = ros::NodeHandle("~") )
  {
    LOCAL_ROS_COMPLIANT_TYPE local;
    
    if( nodeHandle.getParam(path,local) )
    {
      output = static_cast<PARAM_TYPE>(local);
    }
    else
    {
      output = defaultValue;
    }
  }
  template<class PARAM_TYPE>
  void getParamSilent( PARAM_TYPE& output, std::string path, PARAM_TYPE defaultValue, ros::NodeHandle nodeHandle = ros::NodeHandle("~") )
  {
    nodeHandle.param<PARAM_TYPE>(path,output,defaultValue);
  }
  
  /*! Template function to load a parameter with included default value. If it wasn't found and the default is used, a warning is issued.
   * 
   * Function is overloaded for one or two template parameters: the version with two provides the following functionality: If the target value is of a type that
   * is not supported by ROS' getParam functions but can be (static_-)casted from another type that is supported, give the local ros-compliant
   * type as second template argument. Don't forget to check in the output whether the result of the typecast is as expected. If the 
   * type is ros-compliant, no template argument needs to be given as it can be deduced.
   * 
   * ATTENTION: Can't be used if PARAM_TYPE (first template argument type) can't be printed...
   * 
   * @param output (output) Where the parameter will be written
   * @param path Path that will be read from the nodehandle
   * @param defaultValue Default value that will be loaded if not provided
   * @param nodeHandle Node handle (defaults to a private ROS node handle: ros::NodeHandle("~") )
   */
  template<class PARAM_TYPE, class LOCAL_ROS_COMPLIANT_TYPE>
  void getParam( PARAM_TYPE& output, std::string path, PARAM_TYPE defaultValue, ros::NodeHandle nodeHandle = ros::NodeHandle("~") )
  {
    LOCAL_ROS_COMPLIANT_TYPE local;
    if( nodeHandle.getParam(path,local) )
    {
      output = static_cast<PARAM_TYPE>(local);
      //ROS_INFO_STREAM("Loaded parameter '"<<path<<"': '"<<output<<"'.");
    }
    else
    {
      output = defaultValue;
      ROS_WARN_STREAM("Parameter '"<<path<<"' not provided, using default: '"<<output<<"'.");
    }
  }
  template<class PARAM_TYPE>
  void getParam( PARAM_TYPE& output, std::string path, PARAM_TYPE defaultValue, ros::NodeHandle nodeHandle = ros::NodeHandle("~") )
  {
    if( nodeHandle.getParam(path,output) )
    {
      //ROS_INFO_STREAM("Loaded parameter '"<<path<<"': '"<<output<<"'.");
    }
    else
    {
      output = defaultValue;
      ROS_WARN_STREAM("Parameter '"<<path<<"' not provided, using default: '"<<output<<"'.");
    }
  }
  
  /*! Template function to load a parameter with included default value. If it wasn't found the value is not set.
   * 
   * Function is overloaded for one or two template parameters: the version with two provides the following functionality: If the target value is of a type that
   * is not supported by ROS' getParam functions but can be (static_-)casted from another type that is supported, give the local ros-compliant
   * type as second template argument. Don't forget to check in the output whether the result of the typecast is as expected. If the 
   * type is ros-compliant, no template argument needs to be given as it can be deduced.
   * 
   * ATTENTION: Can't be used if PARAM_TYPE (first template argument type) can't be printed...
   * 
   * @param output (output) Where the parameter will be written
   * @param path Path that will be read from the nodehandle
   * @param defaultValue Default value that will be loaded if not provided
   * @param nodeHandle Node handle (defaults to a private ROS node handle: ros::NodeHandle("~") )
   */
  template<class PARAM_TYPE, class LOCAL_ROS_COMPLIANT_TYPE>
  void getParamIfAvailable( PARAM_TYPE& output, std::string path, ros::NodeHandle nodeHandle = ros::NodeHandle("~") )
  {
    LOCAL_ROS_COMPLIANT_TYPE local;
    if( nodeHandle.getParam(path,local) )
    {
      output = static_cast<PARAM_TYPE>(local);
      //ROS_INFO_STREAM("Loaded parameter '"<<path<<"': '"<<output<<"'.");
    }
    else
    {
      ROS_WARN_STREAM("Parameter '"<<path<<"' not provided, using default: '"<<output<<"'.");
    }
  }
  template<class PARAM_TYPE>
  void getParamIfAvailable( PARAM_TYPE& output, std::string path, ros::NodeHandle nodeHandle = ros::NodeHandle("~") )
  {
    if( nodeHandle.getParam(path,output) )
    {
      //ROS_INFO_STREAM("Loaded parameter '"<<path<<"': '"<<output<<"'.");
    }
    else
    {
      ROS_WARN_STREAM("Parameter '"<<path<<"' not provided, using default: '"<<output<<"'.");
    }
  }
  
  /*! Template function to load a parameter with included default value. If it wasn't found the value is not set.
   * 
   * Function is overloaded for one or two template parameters: the version with two provides the following functionality: If the target value is of a type that
   * is not supported by ROS' getParam functions but can be (static_-)casted from another type that is supported, give the local ros-compliant
   * type as second template argument. Don't forget to check in the output whether the result of the typecast is as expected. If the 
   * type is ros-compliant, no template argument needs to be given as it can be deduced.
   * 
   * ATTENTION: Can't be used if PARAM_TYPE (first template argument type) can't be printed...
   * 
   * @param output (output) Where the parameter will be written
   * @param path Path that will be read from the nodehandle
   * @param defaultValue Default value that will be loaded if not provided
   * @param nodeHandle Node handle (defaults to a private ROS node handle: ros::NodeHandle("~") )
   */
  template<class PARAM_TYPE, class LOCAL_ROS_COMPLIANT_TYPE>
  void getParamIfAvailableSilent( PARAM_TYPE& output, std::string path, ros::NodeHandle nodeHandle = ros::NodeHandle("~") )
  {
    LOCAL_ROS_COMPLIANT_TYPE local;
    if( nodeHandle.getParam(path,local) )
    {
      output = static_cast<PARAM_TYPE>(local);
    }
  }
  template<class PARAM_TYPE>
  void getParamIfAvailableSilent( PARAM_TYPE& output, std::string path, ros::NodeHandle nodeHandle = ros::NodeHandle("~") )
  {
    nodeHandle.getParam(path,output);
  }
}