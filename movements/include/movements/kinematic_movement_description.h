/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of movements, a library for representations and calculations of movements in space,

movements is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
movements is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with movements. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "movements/relative_movement.h"
#include "movements/combined_kinematic_movement_description.h"

namespace movements
{


/** class that serves as package wrapper for KinematicMovementDescriptionInstances that naturally need to be moved around as pointers. The use of this class keeps the whole pointer arithmetic internal and thus easens the use. */
class KinematicMovementDescription
{
public:
  class KinematicMovementDescriptionInstance;
  struct PathInfo;
  
  /// constructor
  KinematicMovementDescription(){};
  KinematicMovementDescription( KinematicMovementDescriptionInstance* _to_enwrap );
  
  /** returns the type of the enclosed kinematic movement */
  std::string type();
  
  /** returns the relative movement at time _time
   * @param _time in [s]
   */
  RelativeMovement operator()( double _time );
  
  /** returns a pointer to the internally hold KinematicMovementDescriptionInstance */
  boost::shared_ptr<KinematicMovementDescriptionInstance> operator->();
  
  /** returns a reference to the internally hold KinematicMovementDescriptionInstance */
  KinematicMovementDescriptionInstance& operator*();
  
    
  /** this creates a vector filled with relative movements generated from the kinematic movement description, where the first pose in the vector equals the relative movement generated for time _start_time and the last relative movement the relative movement generated at time t_last, where t_last is the largest time step retrieved by adding _step_size to _start_time that is less or equal _end_time
   * @throws std::invalid_argument if _step_size<=0 or _start_time>_end_time
   * @param _start_time start time for the first pose
   * @param _end_time latest time for the last pose
   * @param _step_size time step size [s]
   */
  std::vector<RelativeMovement> relativePath( double _start_time, double _end_time, double _step_size );
  
  /** this creates a vector filled with poses generated from the kinematic movement description, where the first pose in the vector equals the pose generated for time _start_time and the last pose the pose generated at time t_last, where t_last is the largest time step retrieved by adding _step_size to _start_time that is less or equal _end_time
   * @throws std::invalid_argument if _step_size<=0 or _start_time>_end_time
   * @param _base_pose base pose for the poses that are to be generated
   * @param _start_time start time for the first pose
   * @param _end_time latest time for the last pose
   * @param _step_size time step size [s]
   */
  movements::PoseVector path( movements::Pose _base_pose, double _start_time, double _end_time, double _step_size );
  
  /** creates a relative kinematic event chain where the kinematic movement represented by the class object is prepended to the argument _to_add */
  template<class MovementT>
  CombinedKinematicMovementDescription operator+( MovementT const& _to_add );
private:
  boost::shared_ptr<KinematicMovementDescriptionInstance> enwrapped_kinematic_movement_description_;
};

typedef KinematicMovementDescription KinMove;


/** defines the interface for relative kinematic movement classes */
class KinematicMovementDescription::KinematicMovementDescriptionInstance
{
public:
  /// returns the type of the enclosed kinematic movement
  virtual std::string type()=0;
  
  /** returns the relative movement at time _time */
  virtual RelativeMovement operator()( double _time )=0;
  
  virtual std::vector<RelativeMovement> relativePath( double _start_time, double _end_time, double _step_size );
  
  /** this creates a vector filled with poses generated from the kinematic movement description, where the first pose in the vector equals the pose generated for time _start_time and the last pose the pose generated at time t_last, where t_last is the largest time step retrieved by adding _step_size to _start_time that is less or equal _end_time
   * @throws std::invalid_argument if _step_size<=0 or _start_time>_end_time
   * @param _base_pose base pose for the poses that are to be generated
   * @param _start_time start time for the first pose
   * @param _end_time latest time for the last pose
   * @param _step_size time step size [s]
   */
  virtual movements::PoseVector path( movements::Pose _base_pose, double _start_time, double _end_time, double _step_size );
};

struct KinematicMovementDescription::PathInfo
{
  double start_time;
  double end_time;
};

template<class MovementT>
CombinedKinematicMovementDescription KinematicMovementDescription::operator+( MovementT const& _to_add )
{
  CombinedKinematicMovementDescription kinematic_movement_chain = *this;
  kinematic_movement_chain += _to_add;
  return kinematic_movement_chain;
}

}