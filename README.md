# quadrotor_autonav

## Synopsis

This project provides a navigation package for use with ROS and hector_quadrotor.
Using an already implemented SLAM algorithm, this project aims to create a local map/occupancy grid given an unknown environment and to optimally traverse that environment once the map is fully built.
Currently, this navigation stack uses UCS (uniform cost search) to create and build the map and A* search to optimally find a path to a goal once the map is built.

## Motivation

The main goal of this project is to create a starting point for a real, non-simulated quadrotor build. This simulation is the starting point for a future project to create an indoor quadrotor vehicle, map the environment it is placed in, and to optimally traverse that environment. The absolute end goal would be to map an environment, find some type of object, and move it to a specified goal.

## Installation

**Software Prerequisites**
  + ROS installation (see http://wiki.ros.org/ROS/Installation for installation and setup details)
  + Hector_quadrotor installatdion (see http://wiki.ros.org/hector_quadrotor/Tutorials/Quadrotor%20indoor%20SLAM%20demo for installation and setup details)

**Setup**
  + Create a catkin workspace and place this project within that workspace (see http://wiki.ros.org/catkin/Tutorials/create_a_workspace for help on creating the workspace)
  + Once the workspace is created, run **catkin_make** in the root of that workspace directory. This command is needed anytime the source files are modified. 
  + Place the "bigapartment" folder in the ~/.gazebo/models directory (home/.gazebo/models)

## Usage
  + Open a terminal window and type **roslaunch quadroter_auto_nav big_apartment.launch**
  + Open another terminal window and type **rosrun quadroter_auto_nav main** to start the navigation
  + At this point, the vehicle will is placed in an unknown environment and will perform UCS path planning until the entire map is created. Wait until the environment is mapped until we can optimally traverse it. (Feel free to watch the quadrotor navigate, or go grab a snack/drink as this will take a couple of minutes)
  + Once the map is fully explored, click "Publish Point" in RVIZ. Click a gridpoint in the map, and A* search will be performed to optimally calculate the path and navigate to that goal.

## Examples

When searching for a path, you can visualize all the states expanded by the green marker points as shown:
![Node expansion](images/node_expansion.png?raw=true "State nodes expansion visualization")

To disable the node expansion marker, uncheck the quadcopter_points_States_Expanded option:
![Disable node expansion visualization](images/marker_states_off.png?raw=true "State nodes expansion visualization disabled")

The calculated path is displayed with red marker points:
![Calculated path](images/path_retrieved.png?raw=true "Calculated path visualization")

## Contributors

Chris Allen and Tyler Gauch.

