**This is a first rough readme, will be more formulated out.**

mav_active_3d_planning contains code for  voxblox based active path planning in unknown environments with the goal of optimizing a 3D-reconstruction gain such as exploration and quality while minimizing a cost. 

We provide a framework for buidling and evaluating sampling based, receding horizon planners.

# Table of Contents
**Installation**
* [Installation](#Installation)
* [Dependencies](#Dependencies)
* [Data Repository](#Data-Repository)

**Examples**
* [Configuring a Planner](#Configuring-a-Planner)
* [Run an Experiment](#Run-an-Experiment)

**Documentation and additional Information**
* **Planner Structure**
  * [Main Planner](wiki/Planner-Structure#Main-Planner)
  * [Trajectory Generator](wiki/Planner-Structure#Trajectory-Generator)
  * [Trajectory Evaluator](wiki/Planner-Structure#Trajectory-Evaluator)
  * [Voxblox Server](wiki/Planner-Structure#Voxblox-Server)
* **Planner Design Framework**
  * [Modular Configuration](wiki/Planner-Design-Framework#Modular-Configuration)
  * [Building Planners](wiki/Planner-Design-Framework#Building-Planners)
  * [Contributing Custom Modules](wiki/Planner-Design-Framework#Contributing-Custom-Modules)
* **Running and Evaluating a Simulated Experiment**
  * [Simulation Framework](wiki/Running-and-Evaluating-a-Simulated-Experiment#Simulation-Framework)
  * [Conducting an Experiment](wiki/Running-and-Evaluating-a-Simulated-Experiment#Conducting-an-Experiment)
  * [Results and Monitoring Tools](wiki/Running-and-Evaluating-a-Simulated-Experiment#Results-and-Monitoring-Tools)
* **Code Index**
  * [Module Index](wiki/Code-Index)
  * [Planner Node](wiki/Code-Index#Planner-Node)
  * [Experiment Nodes](wiki/Code-Index#Experiment-Nodes)
  
# Installation
TODO

## Dependencies
To run the mav_active_3d_planning simulation framework, the following packages are required: `gazebo_ros`, `rotors_gazebo`, `mav_nonlinear_mpc`, `mav_lowlevel_attitude_controller`, `voxblox_ros` and `unreal_cv_ros`.

Other modules use `mav_trajectory_generation`.

## Data Repository
Related ressources can be downloaded from [here](https://www.polybox.ethz.ch/index.php/s/6vhPDINcISbEogg). 

# Examples
## Configuring a Planner
TODO

See eg. `cfg/example_config.yaml` ...

## Run an Experiment
TODO

Run `run_experiment.launch` and fiddle with the args I guess ...
