# Information gain based active reconstruction

ig_active_reconstruction is a set of catkin packages for **information gain based active reconstruction** as presented in the paper

*An Information Gain Formulation for Active Volumetric 3D Reconstruction*  
S. Isler, R. Sabzevari, J. Delmerico and D. Scaramuzza (ICRA 2016)  
[Paper: http://rpg.ifi.uzh.ch/docs/ICRA16_Isler.pdf](http://rpg.ifi.uzh.ch/docs/ICRA16_Isler.pdf)  
[Video (Youtube)](https://www.youtube.com/watch?v=ZcJcsoGGqbA&feature=youtu.be) 

The following functionality can be found in the different packages:
* **ig_active_reconstruction** ROS-independent library providing interface definitions and classes commonly used such as *View*, *ViewSpace* as well as, e.g., a simple view planner implementation and utility calculator.
* **ig_active_reconstruction_octomap** ROS-independent interface templates to the [Octomap library](https://octomap.github.io/), templated on the tree type; input class templates e.g. for different [PCL inputs](http://pointclouds.org/); bindings to ig_active_reconstruction that use Octomap as container in a world representation module; ROS bindings and an example of a running world representation ROS node. 
* **ig_active_reconstruction_ros** Interfaces for ROS and ig_active_reconstruction with which any class that inherits from the communication interfaces in ig_active_reconstruction can communicate with other modules that do the same using ROS; conversion between ROS messages and ig_active_reconstruction types
* **example/flying_gazebo_stereo_cam** Example for how to use above classes to control a robot: A free-flying stereo camera for the Gazebo simulation environment and a controller for it that implements a robot interface as defined in ig_active_reconstruction, incl. the ROS binding from ig_active_reconstruction_ros.

## Installation/Dependencies
All packages are written to be compiled using *catkin*, just put them in your catkin workspace. If you are using ROS, most dependencies should already be installed. Packages were tested on Ubuntu 14.04 under ROS Indigo, with gcc 4.7 and 4.8 (we are currently resolving some issues that prevent compilation with gcc 4.9 and 5.0).

* **ig_active_reconstruction:** Eigen
* **ig_active_reconstruction_octomap:** Boost, PCL 1.7+, Eigen, Octomap
* **ig_active_reconstruction_ros:** -
*  **example/flying_gazebo_stereo_cam:** Gazebo, stereo_image_proc

To install all of these dependencies as system packages, run:
```
sudo apt-get install ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-ros-control \
                    ros-indigo-stereo-image-proc ros-indigo-octomap libboost-all-dev \
                    libpcl-1.7-all-dev libeigen3-dev
```

## Running the Example
To get you started we provide the flying_gazebo_stereo_cam example. Note that we provide a Stanford bunny Gazebo model file which you can use to get started. Off course you can also use any other Gazebo model and put it in front of the stereo camera in Gazebo.

To use our model, either copy the "bunny" folder (which contains the collada file, texture and Gazebo model definition) to ~/.gazebo/models, or add its directory to your GAZEBO_MODEL_PATH environmental variable with e.g.:
```
export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/rpg_ig_active_reconstruction/example/flying_gazebo_stereo_cam/model:$GAZEBO_MODEL_PATH
```

If everything compiled, run the following in four different terminals to start the reconstruction procedure:
* **roslaunch flying_gazebo_stereo_cam robot_interface.launch**  
Launches Gazebo and loads the bunny (if you put it in the models folder), starts a viewspace model and a robot interface ROS node offering their services to other ig_active_reconstruction components
* **roslaunch flying_gazebo_stereo_cam flying_gazebo_stereo_cam.launch**
Launches the node that spawns the stereo camera (You should see two small boxes pointing at the origin) in Gazebo. This needs to be started after robot_interface.
* **roslaunch ig_active_reconstruction_octomap octomap_world_representation.launch**  
Launches a world representation ROS node using Octomap as a container and offering information gain calculation capabilities.
* **roslaunch ig_active_reconstruction_ros	basic_view_planner.launch**  
Launches a basic view planner node with a simple command line user interface that allows you to start, pause and stop the procedure.  **To start the demo, press 'g' and then 'Enter' in this terminal.**

Play around with the parameters in the different launch files to see what effect they have.
