The mav_active_3d_planning package is dedicated to the design, evaluation and application of active path planning algorithms for MAVs. We provide a framework for creating, evaluating and employing primarily sampling based, receding horizon algorithms that optimize a gain while minimizing a cost, for example exploration and quality against execution time in the case of autonomous 3D reconstruction. 

# Paper
If you find this package useful for your research, please consider citing:
```
@beingWritten{
  A paper is currently being written.
  The repo is for confidential use only at the moment.
}
```

# Table of Contents
**Installation**
* [Dependencies](#Dependencies)
* [Installation](#Installation)
* [Data Repository](#Data-Repository)

**Examples**
* [Configuring a Planner](#Configuring-a-Planner)
* [Run an Experiment](#Run-an-Experiment)

**Documentation**
* [Planner Structure](https://github.com/ethz-asl/mav_active_3d_planning/wiki/Planner-Structure)
* [Planner Design Framework](https://github.com/ethz-asl/mav_active_3d_planning/wiki/Planner-Design-Framework)
* [Running and Evaluating a Simulated Experiment](https://github.com/ethz-asl/mav_active_3d_planning/wiki/Running-and-Evaluating-a-Simulated-Experiment)
* [Code Index](https://github.com/ethz-asl/mav_active_3d_planning/wiki/Code-Index)

For additional information please see the wiki.

# Installation
## Dependencies
**ROS Packages:**

The mav_active_3d_planning package is divded into separate packages, such that only the dependencies necessary for your application package need to be built.
Packages depend on:
* **core:**
    * `catkin_simple` ([https://github.com/catkin/catkin_simple](https://github.com/catkin/catkin_simple))
    * `glog_catkin` ([https://github.com/ethz-asl/glog_catkin](https://github.com/ethz-asl/glog_catkin))
    * `eigen_catkin` ([https://github.com/ethz-asl/eigen_catkin](https://github.com/ethz-asl/eigen_catkin))
    
* **mav:**
    * `mav_trajectory_generation` ([https://github.com/ethz-asl/mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation))


* **voxblox:**
    * `voxblox` ([https://github.com/ethz-asl/voxblox](https://github.com/ethz-asl/voxblox))

* **app_reconstruction:**
    * `unreal_cv_ros` ([https://github.com/ethz-asl/unreal_cv_ros](https://github.com/ethz-asl/unreal_cv_ros))
    * `rotors_simulator` ([https://github.com/ethz-asl/rotors_simulator](https://github.com/ethz-asl/rotors_simulator))
    * `mav_control_rw` ([https://github.com/ethz-asl/mav_control_rw](https://github.com/ethz-asl/mav_control_rw))

## Installation
Installation instructions on Linux:

Move to your catkin workspace: 
```
cd catkin_ws/src
```
Install using a SSH key: 
```
git clone git@github.com:ethz-asl/mav_active_3d_planning.git
```
Compile everything: 
```
catkin build mav_active_3d_planning
```

## Data Repository
Related ressources, such as experiment scenarios and ground truth point clouds, can be downloaded from [here](https://www.polybox.ethz.ch/index.php/s/6vhPDINcISbEogg). 

# Examples
## Configuring a Planner
The `active_3d_planning_app_reconstruction` is an application package, that builds and launches an active\_3d\_planner.
A verbose example of how planners are created through a config is given in `cfg/example_config.yaml`. The presented planner uses local motion primitives to expand new segments and the number of unknown voxels as gain formulation. To see the planner in action, start an unreal\_cv\_ros game, e.g. CityBuilding, make sure to tab out of game control with Ctrl+Shift+F1 and then run 
```
roslaunch active_3d_planning_app_reconstruction example.launch
```
The planner will be built from the config file and visualized in RVIZ. A useful parameter to set is `verbose_modules: true`, as all available params of all built modules will be printed to console. 

![mav_3d_ex_config](https://user-images.githubusercontent.com/36043993/58561558-aaa84280-8227-11e9-9b89-def052db17a8.png)

A local motion primitve based planner starting exploration.

## Run an Experiment
In order to record data of the example planner, run 
```
roslaunch active_3d_planning_app_reconstruction example.launch data_directory:=/path/to/my/data_dir
```
When the experiment is finished by the time limit of 30 minutes or by pressing Ctrl+C, run 
```
roslaunch mav_active_3d_exploration evaluate_experiment.launch target_directory:=/path/to/my/data_dir gt_file_path:=/path/to/experiment1/gt_surface_pcl.ply
```
When the process is finished, the created data directory contains a folder 'Graphs', containing the evaluation results as well as a folder 'Meshes', which can be visualized using e.g. [CloudCompare](https://www.danielgm.net/cc/). 

![SimulationOverview](https://user-images.githubusercontent.com/36043993/59348747-33d77300-8d18-11e9-935e-d89a3fc64f64.png)
Performance overview of the planner over the course of the simulated experiment.


![PerformanceOverview](https://user-images.githubusercontent.com/36043993/59348802-5d909a00-8d18-11e9-984f-7a1dc7c7a8ba.png)
Distribution of computation time for the different modules. The majority is expended for gain computation (red). Notice that the voxblox map serialization increases as the map grows (dark grey).


![mav_active](https://user-images.githubusercontent.com/36043993/59349935-253e8b00-8d1b-11e9-87d8-6d57463b9596.png)
Final reconstruction mesh and error coloring visualized in CloudCompare.
