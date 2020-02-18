# active\_3d\_planning\_app_reconstruction
Application package for volumetric exploration and 3D reconstruction with MAVs and using voxblox. Contains:
* Launch and configuration files for Simulation
* Automated data collection and processing routines

# Examples
## Configuring a Planner
The `active_3d_planning_app_reconstruction` is an application package, that launches an active\_3d\_planner.
A verbose example of how planner configurations are specified is given in `cfg/planners/example_config.yaml`.
The example planner uses local motion primitives to expand new segments and the number of unknown voxels as gain formulation. 
To see the planner in action, start an unreal\_cv\_ros game, e.g. [CityBuilding](#Data-Repository), make sure to tab out of game control (Alt+Tab for Binary, Ctrl+Shift+F1 for Editor) and then run 
```
roslaunch active_3d_planning_app_reconstruction example.launch
```
The planner will be built from the config file and visualized in RVIZ. 
A useful parameter to set is `verbose_modules: true`, as all available params of all built modules will be printed to console. 

![mav_3d_ex_config](https://user-images.githubusercontent.com/36043993/58561558-aaa84280-8227-11e9-9b89-def052db17a8.png)

A local motion primitive based planner starting exploration.

## Run an Experiment
In order to record data of the example planner, run 
```
roslaunch active_3d_planning_app_reconstruction run_experiment.launch data_directory:=/path/to/my_data_dir
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
