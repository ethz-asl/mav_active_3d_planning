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

**Documentation**

Documentation and additional information can be found in the wiki: 
* [Planner Structure](https://github.com/ethz-asl/asldoc-2018-ma-schmid/wiki/Planner-Structure#Main-Planner)
* [Planner Design Framework](https://github.com/ethz-asl/asldoc-2018-ma-schmid/wiki/Planner-Design-Framework)
* [Running and Evaluating a Simulated Experiment](https://github.com/ethz-asl/asldoc-2018-ma-schmid/wiki/Running-and-Evaluating-a-Simulated-Experiment)
* [Code Index](https://github.com/ethz-asl/asldoc-2018-ma-schmid/wiki/Code-Index)


  
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
