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

**ROS Nodes**
* [planner_node](#planner_node)
* [eval_data_node](#eval_data_node)
* [eval_plotting_node](#eval_plotting_node)
* [eval_voxblox_node](#eval_voxblox_node)

**List of Modules**
* [Default Modules](#Default-Modules)

**Examples**
* [Configuring a Planner](#Configuring-a-Planner)
* [Run an Experiment](#Run-an-Experiment)

# Installation
TODO

## Dependencies
Uses [nanoflann](#https://github.com/jlblancoc/nanoflann) (included as header lib, no need to install).

To run the mav_active_3d_planning simulation framework, the following packages are required: `gazebo_ros`, `rotors_gazebo`, `mav_nonlinear_mpc`, `mav_lowlevel_attitude_controller`, `voxblox_ros` and `unreal_cv_ros`.

Other modules use `mav_trajectory_generation`.

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
* **visualizeTrajectoryValue** Optional. How to display the expected gain in RVIZ.

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
Custom implementations for all module types can easily be added to the framework. Custom modules inherit from a base module type that specifies the interface with other classes. All base types inherit from the `Module` class, which contains utilities to configure and build modules through the `ModuleFactory`. To create a custom module follow these steps:

* **Create your module class** 

All module classes should be organized as follows:
```c++
class MyModule : public ModuleBase {
public:
    // override virtual functions of the base class
    bool moduleBasePureVirtualFunction(TrajectorySegment *root){
        /* do some magic here */
        return success;
    }

protected:
    // All modules need to be friends of the ModuleFactory to allow creation and setup
    friend ModuleFactory;

    // protected default constructor
    MyModule() {}
    
    // Statically register the module to the factory so that it can be constructed
    static ModuleFactory::Registration<MyModule> registration;

    // make the module configurable through the factory (required by Module class)
    void setupFromParamMap(Module::ParamMap *param_map){
        int my_param_default = 1;
        
         // Use the utility function of Module to set params
        setParam<int>(param_map, "my_param", &p_my_param_, my_param_default); 

        // Make sure to propagate the setup through the inheritance chain
        ModuleBase::setupFromParamMap(param_map);
    }

    // guarantee that parameters fulfill required constraints (optional by Module class)
    bool checkParamsValid(std::string *error_message) {
        if (p_my_param <= 0) {
            *error_message = "my_param expected > 0";
            return false;
        }

        // Make sure to propagate the validation check through the inheritance chain
        return ModuleBase::checkParamsValid(error_message);
    }

    // params
    int p_my_param_;
};

// make sure the registration is initialized with a unique name (i.e. the name of the class)
ModuleFactory::Registration<MyModule> MyModule::registration("MyModule");

```
If your module uses other modules, e.g. to be used in a chain of decorators, adapt these functions as follows:
```c++
class MyDecoratorModule : public ModuleBase {
public:
    bool moduleBasePureVirtualFunction(TrajectorySegment *root){
        // pass the request down the chain
        return following_module->moduleBasePureVirtualFunction(root);
    }

protected:
    void setupFromParamMap(Module::ParamMap *param_map){
        // Submodules are created using this formalism
        std::string args;   // the module args need to be specifiable
        std::string param_ns = (*param_map)["param_namespace"]; // default extends the parent namespace
        setParam<std::string>(param_map, "following_module_args", &args, param_ns + "/following_module");
        following_module_ = ModuleFactory::Instance()->createModuleBase(args, verbose_modules_, parent_);
    }

    // Modules are unique_ptrs
    std::unique_ptr<ModuleBase> following_module_;
};
```

* **Add some doc..?**

For inspiration maybe look through some of the default modules.

# Running and Evaluating a Simulated Experiment
To test and compare the performance of planners, we provide a simulation environment and evaluation tools.

## Simulation Framework
The simulation environment is similar to the one presented in `unreal_cv_ros`. We use `gazebo` to model the MAV physics and `unreal_cv_ros` for perception and collision detection. The planner node requests trajectories, which are followed using the `mav_nonlinear_mpc` and `mav_lowlevel_attitude_controller` controllers. During execution, `unreal_cv_ros` produces pointclouds. These are integrated by the `voxblox` server into a map, based upon which the planner then proposes new trajectories.

## Conducting an Experiment
We provide utility ros nodes and launch scripts, that make conducting and evaluating experiments as simple as possible:
* Start the unreal game map on which the experiment takes place.
* Run `run_experiment.launch`

  This launchfile coordinates the startup of all components of the simulation framework, as well the launch and termination of the planner. Furthermore, a data directory is created that stores the generated information.
* After termination, run `evaluate_experiment.launch`

  This launchfile takes a data directory, computes different evaluation metrics and produces graphs.
* To reenact the behaviour of a planner, run `replay_experiment.launch`
  
  This launchfile replays a rosbag of the planner visualization topics, can also use fast forward etc.

## Results and Monitoring Tools
During simulation, the planner and eval_data_node produce the following raw data (in a predefined structure):
* The current voxblox map is periodically saved
* together with the elapsed simulated time, wall time, cpu time and the number of integrated pointclouds
* After every replanning step, the cpu time per function, the total simulated and cpu time as well as the number of considered TrajectorySegments is recorded.
* The complete rosparam configuration is dumped, such that the planner and other settings can be recovered.
* A rosbag of the visualization topics is recorded in "tmp_bags". (When evaluation, it is looked up and moved to the right folder)

During evaluation, the eval_plotting_node and eval_voxblox_node use this data to compare the saved maps against a ground truth pointcloud and compute the following metrics:
* Mean and standard deviation of the absolute difference between constructed map and ground truth
* Percentage of the ground truth pointcloud that was discovered (is contained in the map).

Additionally, the following visualizations are produced:
* The folder meshes contains the periodically saved mesh instances to reconstruct exploration progress.
* Additionally, every mesh is colored according to the absolute difference to ground truth, where dark green represents 0, red the maximum error threshold (by default 2x the voxel size), and gray voxel outside this threshold (these are also not included in the error computation).
* The folder graphs contains a 'SimulationOverview.png', containing line graphs for reconstruction quality and exploration progress over time,
* And a 'PerformanceOverview.png', showing where the computation time was allocated and how much total computation time was consumed.

All produced data is stored in csv files, which can be used for further visualization. The data_log.txt contains additional information about the experiment execution and processing.

# ROS Nodes
TODO: Need some proper doc here (topics, params, ...)

## planner_node
This is the node that contains and runs the main planner (so essentially every planner).

## eval_data_node
This node helps starting and stopping experiments and records raw data.

## eval_plotting_node
This node manages evaluation and produces graphs.

## eval_voxblox_node
Ros encapsulation for c++ voxblox code. Is called by the eval_plotting_node during evaluation.


# Default Modules
There is a lot of modules already, some doc wouldn't be bad ...


# Examples
## Configuring a Planner
TODO

See eg. `cfg/example_config.yaml` ...

## Run an Experiment
TODO

Run `run_experiment.launch` and fiddle with the args I guess ...
