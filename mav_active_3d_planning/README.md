mav_active_3d_planning contains code for  voxblox based active path planning in unknown environments with the goal of optimizing a 3D-reconstruction gain such as exploration and quality while minimizing a cost. 
This is a first rough readme.

# Table of Contents
**Planner Structure**
* [Main Planner](#Main_Planner)
* [Trajectory Generator](#Trajectory-Generator)
* [Trajectory Evaluator](#Trajectory-Evaluator)
* [Voxblox Server](#MVoxblox-Server)

**ROS nodes**
* [planner_node](#planner_node)

**Examples**
* [Proof of concept](#Proof-of-concept)

## Dependencies
voxblox, 

# Planner Structure
The planner is structured in a modular setup to provide a clear interface for customization. It consists of 4 essential ingredients:

## Main Planner
This is the master node containing the 3 other classes as members. In a receding horizon fashion, a tree of possible trajectory segments is continuosly expanded and expected costs and gains are computed. 
Once the current trajectory segment has finished executing, the next best adjacent segment is published and the trajectory tree updated.

## Trajectory Generator
The trajectory generator is responsible for expanding the trajectory tree. 
To guarantee constraint satisfaction, the proposed new trajectories need to fulfill all system (such as maximum thrusts, rates, ...) and environment (such as collision, ...) constraints.
All constraints therefore need to be incorporated in the trajectory generator. Custom, derived trajectory generators need to implement the following virtual functions:
* **selectSegment** Expansion policy where to expand the current tree.
* **expandSegment** Add adjacent trajectory segments to the target segment.
* **setParamsFromRos** Initialization using the main planner's nodehandle.

## Trajectory Evaluator
The trajectory evaluator computes expected gains, costs and final values for different trajectory segments.
Custom, derived trajectory generators need to implement the following virtual functions:
* **computeGain** Compute the expected gain from executing a trajectory segment.
* **computeCost** Compute the expected cost from executing a trajectory segment.
* **computeValue** Assign the final value for a trajectory segment, usually f(gain, cost, ...).
* **selectNextBest** Execution Policy for available segments.
* **setParamsFromRos** Initialization using the main planner's nodehandle.

## Voxblox Server
The main planner includes a [voxblox](https://github.com/ethz-asl/voxblox) server, which is updated from a separate voxblox node and contains all information about the environment. 
The server is made available to the trajectory generator and evaluator for constraint satisfaction, gain computation and more.

# ROS nodes
## planner_node
This ros node that contains and runs the main planner.

# Examples
## Proof of concept
To see the naive implementation in action run `roslaunch mav_active_3d_planning full_test.launch`. 
The planner will periodically display all candidate trajectory segments in the tree, colored according to the relative value (from red to green being lowest to highest). 
Furthermore, the expected gain for the currently executing segment is displayed as yellow voxels. 

The sampled trajectories are linear, the gain is the number of unobserved voxels and the cost the execution time of the trajectory. 
This was tested on the RealisticRendering demo using the fast-plugin.
