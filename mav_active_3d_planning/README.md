**This is a first rough readme, will be more formulated out.**

mav_active_3d_planning contains code for  voxblox based active path planning in unknown environments with the goal of optimizing a 3D-reconstruction gain such as exploration and quality while minimizing a cost. 

We provide a framework for buidling and evaluating sampling based, receding horizon planners.

# Table of Contents
**Installation**
* [Installation](#Installation)
* [Dependencies](#Dependencies)
* [Data Repository](#Data-Repository)

**Planner Structure**
* [Main Planner](#Main_Planner)
* [Trajectory Generator](#Trajectory-Generator)
* [Trajectory Evaluator](#Trajectory-Evaluator)
* [Voxblox Server](#Voxblox-Server)

**Planner Design Framework**
* [Modular Configuration](#Modular-Configuration)
* [Building Planners](#Building-Planners)
* [Contributing Custom Modules](#Contributing-Custom-Modules)

**Running and Evaluating a Simulated Experiment**
* [Simulation Framework](#Simulation-Framework)
* [Conducting an Experiment](#Conducting-an-Experiment)
* [Results and Monitoring Tools](#Results-and-Monitoring-Tools)

**ROS nodes**
* [planner_node](#planner_node)

**Examples**
* [Configuring a Planner](#Configuring-a-Planner)
* [Run an Experiment](#Run-an-Experiment)

# Installation

Coming...

## Dependencies
To run the mav_active_3d_planning simulation framework, the following packages are required: `gazebo_ros`, `rotors_gazebo`, `mav_nonlinear_mpc`, `mav_lowlevel_attitude_controller`, `voxblox_ros` and `unreal_cv_ros`.

## Data Repository
Related ressources can be downloaded from [here](https://www.polybox.ethz.ch/index.php/s/6vhPDINcISbEogg). 


# Planner Structure
In this repository we propose a framework for sampling based receding horizon planning. For this purpose we provide a common scaffolding that structures all planners. A planner consists of 4 essential ingredients:

## Main Planner
This is the master node containing the 3 other classes as members. In a receding horizon fashion, a tree of possible trajectory segments is continuosly expanded and expected costs and gains are computed. 
Once the current trajectory segment has finished executing, the next best adjacent segment is published and the trajectory tree updated. Furthermore, the main planner contains a backtracker, which tells the planner what to do in case it gets stuck somewhere.

## Trajectory Generator
The trajectory generator is responsible for expanding the trajectory tree. 
To guarantee constraint satisfaction, the proposed new trajectories need to fulfill all system (such as maximum thrusts, rates, ...) and environment (such as collision, ...) constraints.
All constraints therefore need to be incorporated in the trajectory generator. Trajectory generators need to implement the following virtual functions:
* **selectSegment** Expansion policy where to expand the current tree.
* **expandSegment** Add adjacent trajectory segments to the target segment.
* **updateSegments** Whether and how to update existing segments when a new trajectory is executed.

## Trajectory Evaluator
The trajectory evaluator computes expected gains, costs and final values for different trajectory segments.
Trajectory generators need to implement the following virtual functions:
* **computeGain** Compute the expected gain from executing a trajectory segment.
* **computeCost** Compute the expected cost from executing a trajectory segment.
* **computeValue** Assign the final value for a trajectory segment, usually f(gain, cost, ...).
* **selectNextBest** Policy for executing the next segment.
* **updateSegments** Whether and how to update existing segments when a new trajectory is executed.

## Voxblox Server
The main planner includes a [voxblox](https://github.com/ethz-asl/voxblox) server, which is updated from a separate voxblox node and contains all information about the environment. 
The server is made available to the trajectory generator and evaluator for constraint satisfaction, gain computation and more.


# Planner Design Framework
Many sampling based planners make use of similar functionalities or approaches. To make building new planners as easy as possible, we provide a modular object-oriented framework for creating new planners.

## Modular Configuration
All planners have a common structure as is presented [above](#Planner-Structure). The core components of TrajectoryGenerators and TrajectoryEvaluators are supposed to have a flexible enough interface to allow for a large variety of planners. However, they don't need to implement every single of their virtual functions. Each of these functions has different implementations encapsulated as modules, allowing reusage of common implementations. All functions, which are not explicitely overridden, will be built from these modular building blocks.

Modules are specified and configured via the ros parameter server, where the main planner and other submodules forward their namespace plus the respective module namespace to the ModuleFactory. Every module needs to specify a "type" parameter, according to which the desired classes are instantiated. They are then individually configured through the ros parameter server.

## Building Planners
This object-oriented modular approach has the advantage, that planners consisting of already existing modules can be built at run time via a ros parameter configuration. No more recompiling for testing new planners! The recommended way is to load a "my_config.yaml" into the planner node. An example configuration is given [here](#Configuring-a-Planner). The default namespaces for modules, relative to the planner node, are as follows:
```yaml
trajectory_generator:
  type: "MyTrajectoryGenerator"
  segment_selector:
    type: "MyExpansionPolicy"
  generator_updater:
    type: "MyUpdateStrategy"
    
trajectory_evaluator:
  type: "MyTrajectoryEvaluator"
  cost_computer:
    type: "MyCost"
  value_computer:
    type: "MyValueFunc"
  next_selector:
    type: "MyNextBestSelection"
  evaluator_updater:
    type: "MyUpdateStrategy"
    
back_tracker:
  type: "DontGetStuck"
```

Notice that for some functionalities multiple modules can be combined using the Decorator Pattern:
```yaml
evaluator_updater:
  type: "Operation1"
  following_updater:
    type: "Operation2"
    following_updater:
      ...
```

## Contributing Custom Modules
Custom implementations for all module types can easily be added to the framework:
* Create a "my_module.cpp" in the respective module directory.
* Use the corresponding namespace, inherit from the base module class and implement the new functionality.
* In "module_factory.cpp" include your new module file and add the instantiation to the respective type switch.

  **Note: Modules are to be included and created ONLY through the ModuleFactory!**
* Add some doc..?
* Have fun :)

For an idea of how certain module types are generally structured, have a look through the default modules.

# Running and Evaluating a Simulated Experiment

## Simulation Framework

## Conducting an Experiment

## Results and Monitoring Tools



# ROS nodes
## planner_node
This ros node that contains and runs the main planner.

# Examples
## Configuring a Planner
## Run an Experiment
