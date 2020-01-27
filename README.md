# mav\_active\_3d\_planning
**mav\_active\_3d\_planning** is a modular framework for online informative path planner (IPP) design. 
We provide a modular framework for creating, evaluating and employing primarily sampling based, receding horizon algorithms that optimize a gain while minimizing a cost.

Online-IPP for **Exploration** (left), **3D Reconstruction** (right) & **more**.
![git_gif](https://user-images.githubusercontent.com/36043993/72073736-cbbe2f00-32f0-11ea-977a-dbe7e7a05098.gif)

# Table of Contents
**Credits**
* [Paper and Video](#Paper-and-Video)

**Setup**
* [Packages](#Packages)
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

For additional information please see the [wiki](https://github.com/ethz-asl/mav_active_3d_planning/wiki).

# Credits
## Paper and Video
If you find this package useful for your research, please consider citing our paper:

* Lukas Schmid, Michael Pantic, Raghav Khanna, Lionel Ott, Roland Siegwart, and Juan Nieto, "**An Efficient Sampling-based Method for Online Informative Path Planning in Unknown Environments**", in *IEEE Robotics and Automation Letters*, 2020

```bibtex
@ARTICLE{active_3d_planning,
  author={L. M. {Schmid} and M. {Pantic} and R. {Khanna} and L. {Ott} and R. {Siegwart} and J. {Nieto}},
  journal={IEEE Robotics and Automation Letters},
  title={An Efficient Sampling-based Method for Online Informative Path Planning in Unknown Environments},
  year={2020},
  volume={},
  number={},
  pages={1-1},
  keywords={Motion and Path Planning;Aerial Systems;Perception and Autonomy;Reactive and Sensor-Based Planning},
  doi={10.1109/LRA.2020.2969191},
  ISSN={2377-3774},
  month={},}
```

The planner presented in the paper is given in `active_3d_planning_app_reconstruction/cfg/planners/reconstruction_planner.yaml`.
A video of the approach is available [here](https://www.youtube.com/watch?v=lEadqJ1_8Do).


# Setup
## Packages
The mav_active_3d_planning package is divided into separate packages, such that only the dependencies necessary for your application package need to be built.

Although packages are organized for the catkin workflow, the *core* package can be built as a stand-alone library for non-ROS use. All packages with a short description are listed below.

## Dependencies
Packages and their dependencies:
* **core:**
 
   Central logic of active\_3d\_planners. Dependencies:
    * `catkin_simple` ([https://github.com/catkin/catkin_simple](https://github.com/catkin/catkin_simple))
    * `glog_catkin` ([https://github.com/ethz-asl/glog_catkin](https://github.com/ethz-asl/glog_catkin))
    * `eigen_catkin` ([https://github.com/ethz-asl/eigen_catkin](https://github.com/ethz-asl/eigen_catkin))
    
* **ros:** 

   Interface to ROS for the general active\_3d\_planner and ROS specific modules.

* **mav:** 

   Modules and interfaces specific to Micro Aerial Vehicles (MAV), using ROS. Dependencies:
    * `mav_trajectory_generation` ([https://github.com/ethz-asl/mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation))

* **voxblox:**

   Using voxblox as map representation and modules specific to voxblox. Dependencies:
    * `voxblox` ([https://github.com/ethz-asl/voxblox](https://github.com/ethz-asl/voxblox))

* **app_reconstruction:**

   Application package for autonomous 3D reconstruction with MAVs, including automated simulation and evaluation routines. Dependencies:
    * `unreal_cv_ros` ([https://github.com/ethz-asl/unreal_cv_ros](https://github.com/ethz-asl/unreal_cv_ros))
    * `rotors_simulator` ([https://github.com/ethz-asl/rotors_simulator](https://github.com/ethz-asl/rotors_simulator))
    * `mav_control_rw` ([https://github.com/ethz-asl/mav_control_rw](https://github.com/ethz-asl/mav_control_rw))

## Installation
Installation instructions for Linux.

**Prerequisites**

1. If not already done so, install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full is recommended).

2. If not already done so, create a catkin workspace with [catkin tools](https://catkin-tools.readthedocs.io/en/latest/):

```shell script
sudo apt-get install python-catkin-tools
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/melodic  # exchange melodic for your ros distro if necessary
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel
```

**Installation**

1. Move to your catkin workspace: 
```shell script
cd ~/catkin_ws/src
```

2. Install system dependencies: 
```shell script
sudo apt-get install python-wstool python-catkin-tools
```

3. Download repo using a SSH key or via HTTPS: 
```shell script
git clone git@github.com:ethz-asl/mav_active_3d_planning.git # SSH
git clone https://github.com/ethz-asl/mav_active_3d_planning.git # HTTPS
```

4. Download and install the dependencies of the packages you intend to use.

   * **Full Install:** dependencies of **all** packages can be installed using rosinstall:
   ```shell script
   # system dependencies, replace melodic with your ros distro if necessary:
   sudo apt-get install ros-melodic-cmake-modules ros-melodic-control-toolbox ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink ros-melodic-geographic-msgs autoconf libyaml-cpp-dev protobuf-compiler libgoogle-glog-dev liblapacke-dev libgeographic-dev
   pip install future unrealcv

   # If you already intialized ws tool use 'wstool merge -t'
   wstool init . ./mav_active_3d_planning/mav_active_3d_planning_ssh.rosinstall # SSH
   wstool init . ./mav_active_3d_planning/mav_active_3d_planning_https.rosinstall # HTTPS
   wstool update
   ```
   * **Partial Install:** Install dependencies of the packages you intend to use ([listed above](#Dependencies)) and remove unwanted packages from `mav_active_3d_planning/package.xml` as well as their source folders.

5. Source and compile: 
```shell script
source ../devel/setup.bash
catkin build mav_active_3d_planning # Builds this package only
catkin build # Builds entire workspace, recommended for full install.
```

## Data Repository
Related ressources, such as experiment scenarios and ground truth point clouds, can be downloaded from [here](https://www.polybox.ethz.ch/index.php/s/6vhPDINcISbEogg). 

# Examples
## Configuring a Planner
The `active_3d_planning_app_reconstruction` is an application package, that launches an active\_3d\_planner.
A verbose example of how planner configurations are specified is given in `cfg/planners/example_config.yaml`.
The example planner uses local motion primitives to expand new segments and the number of unknown voxels as gain formulation. 
To see the planner in action, start an unreal\_cv\_ros game, e.g. CityBuilding, make sure to tab out of game control with Ctrl+Shift+F1 and then run 
```
roslaunch active_3d_planning_app_reconstruction example.launch
```
The planner will be built from the config file and visualized in RVIZ. 
A useful parameter to set is `verbose_modules: true`, as all available params of all built modules will be printed to console. 

![mav_3d_ex_config](https://user-images.githubusercontent.com/36043993/58561558-aaa84280-8227-11e9-9b89-def052db17a8.png)

A local motion primitve based planner starting exploration.

## Run an Experiment
In order to record data of the example planner, run 
```
roslaunch active_3d_planning_app_reconstruction run_experiment.launch planner_general_config:=
planners/example_config.yaml data_directory:=/path/to/my_data_dir
```
This will collect and store raw data in a new folder in `my_data_dir`.
When the experiment has finished by time limit (30 minutes) or by pressing Ctrl+C, run 
```
roslaunch active_3d_planning_app_reconstruction evaluate_experiment.launch target_directory:=
/path/to/my_data_dir gt_file_path:=/path/to/CityBuilding/gt_surface_pcl.ply
```
to evaluate the raw data.
When the process is finished, the created data directory contains a folder 'Graphs', containing the evaluation results as well as a folder 'Meshes', which can be visualized using e.g. [CloudCompare](https://www.danielgm.net/cc/). 

![SimulationOverview](https://user-images.githubusercontent.com/36043993/59348747-33d77300-8d18-11e9-935e-d89a3fc64f64.png)
Performance overview of the planner over the course of the simulated experiment.


![PerformanceOverview](https://user-images.githubusercontent.com/36043993/59348802-5d909a00-8d18-11e9-984f-7a1dc7c7a8ba.png)
Distribution of computation time for the different modules. The majority is expended for gain computation (red). Notice that the voxblox map serialization increases as the map grows (dark grey).


![mav_active](https://user-images.githubusercontent.com/36043993/59349935-253e8b00-8d1b-11e9-87d8-6d57463b9596.png)
Final reconstruction mesh and error coloring visualized in CloudCompare.
