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
#include "movements/geometry_pose.h"
#include <deque>

namespace movements
{
  
class CombinedRelativeMovement;
class KinematicMovementDescription;

/// class to hold a series of fixed and kinematic relative movements
class CombinedKinematicMovementDescription
{
public:
  CombinedKinematicMovementDescription();
  
  /** returns the combined relative movement chain for time _time
   * @param _time in [s]
   */
  CombinedRelativeMovement operator()( double _time );
  
    
  /** this creates a vector filled with combined relative movements generated from the combined kinematic movement description, where the first combined relative movement in the vector equals the combined relative movement generated for time _start_time and the last combined relative movement the relative movement generated at time t_last, where t_last is the largest time step retrieved by adding _step_size to _start_time that is less or equal _end_time
   * @throws std::invalid_argument if _step_size<=0 or _start_time>_end_time
   * @param _start_time start time for the first pose
   * @param _end_time latest time for the last pose
   * @param _step_size time step size [s]
   */
  std::vector<CombinedRelativeMovement> relativePath( double _start_time, double _end_time, double _step_size );
  
  /** this creates a vector filled with poses generated from the combined kinematic movement description, where the first pose in the vector equals the pose generated for time _start_time and the last pose the pose generated at time t_last, where t_last is the largest time step retrieved by adding _step_size to _start_time that is less or equal _end_time
   * @throws std::invalid_argument if _step_size<=0 or _start_time>_end_time
   * @param _base_pose base pose for the poses that are to be generated
   * @param _start_time start time for the first pose
   * @param _end_time latest time for the last pose
   * @param _step_size time step size [s]
   */
  movements::PoseVector path( movements::Pose _base_pose, double _start_time, double _end_time, double _step_size );
  
  /** replaces the current relative kinematic movement chain represented by the object with _to_equal */
  CombinedKinematicMovementDescription& operator=( CombinedRelativeMovement const& _to_equal );
  /** replaces the current relative kinematic movement chain represented by the object with _to_equal as the one, single chain element */
  CombinedKinematicMovementDescription& operator=( RelativeMovement const& _to_equal );
  /** replaces the current relative kinematic movement chain represented by the object with _to_equal as the one, single chain element */
  CombinedKinematicMovementDescription& operator=( KinematicMovementDescription const& _to_equal );
    
  
  /** creates a new combined relative kinematic movement with the same relative kinematic movement chain as the current object but with _to_add appended */
  CombinedKinematicMovementDescription operator+( CombinedRelativeMovement const& _to_add );  
  /** creates a new combined relative kinematic movement with the same relative kinematic movement chain as the current object but with _to_add appended */
  CombinedKinematicMovementDescription operator+( RelativeMovement const& _to_add );
  /** creates a new combined relative kinematic movement with the same relative kinematic movement chain as the current object but with _to_add appended */
  CombinedKinematicMovementDescription operator+( KinematicMovementDescription const& _to_add );
  /** creates a new combined relative kinematic movement with the same relative kinematic movement chain as the current object but with _to_add appended */
   CombinedKinematicMovementDescription operator+( CombinedKinematicMovementDescription const& _to_add );
    
  
  /** appends _to_add to the internal relative kinematic movement chain */
  CombinedKinematicMovementDescription& operator+=( CombinedRelativeMovement const& _to_add );  
  /** appends _to_add to the internal relative kinematic movement chain */
  CombinedKinematicMovementDescription& operator+=( RelativeMovement const& _to_add );
  /** appends _to_add to the internal relative kinematic movement chain */
  CombinedKinematicMovementDescription& operator+=( KinematicMovementDescription const& _to_add );
  /** appends _to_add to the internal relative kinematic movement chain */
  CombinedKinematicMovementDescription& operator+=( CombinedKinematicMovementDescription const& _to_add );
private:
  std::deque< std::pair<int,RelativeMovement> > relative_movement_queue_; /// queue for relative movements - the integer is needed to clarify the order of evaluation between the two queues: since elements can only be added to the chain, not subtracted, it is the combined size of the two chains at the time of adding the element, ie an element with a lower associated number takes precedence over one with a higher number
  std::deque< std::pair<int,KinematicMovementDescription> > kinematic_movement_queue_; /// queue for kinematic movement descriptions
  
  /** returns the total number of relative movements currently added (kinematic included */
  unsigned int nrOfMovements();
};

typedef CombinedKinematicMovementDescription CombKinMove;
  
} 
