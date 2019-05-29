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
* [Planner Structure](https://github.com/ethz-asl/asldoc-2018-ma-schmid/wiki/Planner-Structure)
* [Planner Design Framework](https://github.com/ethz-asl/asldoc-2018-ma-schmid/wiki/Planner-Design-Framework)
* [Running and Evaluating a Simulated Experiment](https://github.com/ethz-asl/asldoc-2018-ma-schmid/wiki/Running-and-Evaluating-a-Simulated-Experiment)
* [Code Index](https://github.com/ethz-asl/asldoc-2018-ma-schmid/wiki/Code-Index)

For additional information and see the wiki.

  
# Installation
Installation instructions on Linux:

Move to your catkin workspace: 
```
cd catkin_ws/src
```
Install using a SSH key: 
```
git clone git@github.com:ethz-asl/asldoc-2018-ma-schmid.git
```
Compile: 
```
catkin build mav_active_3d_planning
```

## Dependencies
To run the mav_active_3d_planning simulation framework, the following packages are required: `gazebo_ros`, `rotors_gazebo`, `mav_nonlinear_mpc`, `mav_lowlevel_attitude_controller`, `voxblox_ros` and `unreal_cv_ros`.

To build all modules, the following module dependencies are reuqired: `mav_trajectory_generation`.

## Data Repository
Related ressources can be downloaded from [here](https://www.polybox.ethz.ch/index.php/s/6vhPDINcISbEogg). 

# Examples
## Configuring a Planner
A verbose example of how to build a planner is given in `cfg/example_config.yaml`. The presented planner uses local motion primitives to expand new segments and the number of unknown voxels as gain formulation. To see the planner in action, start an unreal\_cv\_ros game, e.g. Experiment1, make sure to tab out of game control with Ctrl+Shift+F1 and then run `roslaunch mav_active_3d_planning example.launch`. The planner will be built from the config file and visualized in RVIZ. A useful parameter to set is `verbose_modules: true`, as all available params of all built modules will be printed to console. 

![mav_3d_ex_config](https://user-images.githubusercontent.com/36043993/58561558-aaa84280-8227-11e9-9b89-def052db17a8.png)
A local motion primitve based planner starting exploration.

## Run an Experiment
In order to record data of the example planner, run `roslaunch mav_active_3d_planning example.launch record_data:=true data_directory:=/path/to/my/data_dir`. When the experiment is finished by the time limit of 30 minutes or by pressing Ctrl+C, run 
`roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=/path/to/my/data_dir gt_file_path:=/path/to/experiment1/gt_surface_pcl.ply`. When the process is finished, the created data directory contains a folder 'Graphs', containing the evaluation results as well as a folder 'Meshes', which can be visualized using e.g. [CloudCompare](https://www.danielgm.net/cc/). 

TODO: imgs
