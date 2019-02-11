unreal_cv_ros is a package to allow ROS based simulation of a MAV equipped with a 3D-reconstruction sensor. The simulation is performed inside an [unreal engine 4](https://www.unrealengine.com/en-US/what-is-unreal-engine-4) game. The node-game communcation is carried out utilizing the [unrealcv](https://github.com/unrealcv/unrealcv) computer vision plugin for unreal engine 4 (UE4).

# Table of Contents
**Installation**
* [Installation](#Installation)
* [Dependencies](#Dependencies)
* [Data Repository](#Data-Repository)

**ROS nodes**
* [unreal_ros_client](#unreal_ros_client)
* [sensor_model](#sensor_model)
* [simulation_manager](#simulation_manager)

**Working with Unreal**
* [Unrealcv Plugin Setup](#Unrealcv-Plugin-Setup)
* [Creating UE4 worlds](#Creating-UE4-worlds)
* [When to use which mode](#When-to-use-which-mode)
* [Pawns and Cameras](#Pawns-and-Cameras)
* [Custom collision and camera settings](#Custom-collision-and-camera-settings)
* [Static mesh collision](#Static-mesh-collision)
* [Producing ground truth pointclouds](#Producing-ground-truth-pointclouds)
* [The Unreal Coordinate System](#The-Unreal-Coordinate-System)

**Examples**
* [Run in test mode](#Run-in-test-mode)
* [Run with MAV in standard mode](#Run-with-MAV-in-standard-mode)

**Troubleshooting**
* [Frequent Issues](#Troubleshooting)

# Installation
Install as every other ROS package...

## Dependencies
The unreal_ros_client node depends on the unrealcv python library `pip install unrealcv`.

## Data Repository
Related ressources can be downloaded [here](https://www.polybox.ethz.ch/index.php/s/6vhPDINcISbEogg). This repo was developped and tested with unrealcv v0.3.10 on Unreal Engine 4.16.3.

# ROS nodes
## unreal_ros_client
This node manages the unrealcv client and the connection with a running UE4 game. It sets the MAV position and orientation within the unreal game and produces the images and camera calibration used by a 3D sensor model.

### Parameters
* **mode** In which mode the client is operated. Currently implemented are:
  * **test** Navigate the MAV manually in the unreal game. The client will periodically publish the sensor data.
  * **standard** The camera pose is set based on the `/odometry` topic and images are taken. Uses the default unrealcv plugin, operates at ~1 up to 2 Hz.
  * **fast** Similar to standard, but requires a custom unrealcv command (See [Unrealcv Plugin Setup](#Unrealcv-Plugin-Setup)).  Operates at ~3 up to 5 Hz.
  
  Default is 'standard'.
* **collision_on** Set to true to check for collision in the unreal game. Set to false to set the camera anyway. May result in rendering artifacts if the camera overlaps with objects. Default is true.
* **collision_tol** This parameter only shows in `standard` mode. Collision warnings are triggered if the requested and realized position are further apart than this threshold (in unreal units, default unit is cm). Default is 10.
* **publish_tf** If true, the client pulishes a tf-transform of the camera pose for every taken image with a matching timestamp. Default is False.
* **slowdown** Artificially slows down the time between setting the pose and taking images to give unreal engine more time to render the new view. Slowdown is expected as wait duration in seconds wall-time. Default is 0.0.
* **Camera Parameters:** To change the resolution and field of view (FOV) of the camera, the [unrealcv configuration file](http://docs.unrealcv.org/en/master/plugin/config.html) needs to be changed. The relevant path is displayed when the unreal_ros_client node is launched. When the client is setup, these values are published as 'camera_params' on the ros parameter server for other nodes to access them.

### Input Topics
* **odometry** of type `nav_msgs.msg/Odometry`. Set the camera pose w.r.t. its position and yaw at the connection of the client. Does not appear in `test` mode.

### Output Topics
* **ue_sensor_raw** of type `unreal_cv_ros.msg/UeSensorRaw`. The output of the in-game image capture, containing a color and a depth image encoded as npy binaries.


## sensor_model
This node converts the UE client output into a pointcloud for further processing and artificially simulates the behaviour of a 3D sensor (e.g. a stereo reconstruction pipeline).

### Parameters
* **model_type** Which sensor to simulate. Currently implemented are: 
  * **ground_truth:** Produces the ground truth pointcloud without additional processing. 
  
  Default is 'ground_truth'.
* **camera_params_ns** Namespace where to read the unreal camera parameters from, which are expected as {height, width, focal_length}. Notice that the sensor_model waits until the camera params are set on the ros parameter server (e.g. from the unreal_ros_client). Default is 'camera_params'.
* **maximum_distance** All points whose original ray length is beyond maximum_distance are removed from the pointcloud. Set to 0 to keep all points. Default is 0.0.
* **flatten_distance** Sets the ray length of every point whose ray length is larger than flatten_distance to flatten_distance. Set to 0 to keep all points unchanged. Default is 0.0.

### Input Topics
* **ue_sensor_raw** of type `unreal_cv_ros.msg/UeSensorRaw`. Output of the unreal\_ros\_client that is to be further processed.

### Output Topics
* **ue_sensor_out** of type `sensor_msgs.msg/PointCloud2`. Resulting pointcloud after applying the simulated sensing pipeline.


## simulation_manager
This node is used to coordinate the full MAV simulation using gazebo as a physics engine and an unreal game for perception and collision modeling. It is used to coordinate simulation setup, monitor or supervise the unreal_ros vision pipeline and to produce simulation data.

### Parameters
* **ns_gazebo** Namespace of gazebo, including the node name. Default is '/gazebo'.
* **ns_mav** Namespace of the MAV, which is expected to end with the actual MAV name. Default is '/firefly'.
* **ns_planner** Namespace of the planner, including the node name. Default is '/firefly/planner_node'.
* **delay** Additional waiting time in seconds before launching the planner to give everything time to settle. Default is 0.0.
* **monitor** Set to true to measure the unreal vision pipeline's performance. Default is False.
* **horizon** How many datapoints are kept and considered for the performance measurement. Only available if monitor is true. Default is 10.
* **evaluate** Set to true to create simulation data and periodically store the voxblox map. Default is false.
* **eval_directory** Where to create the data folder. This param is required to be set and point to an existing directory. Only available if evaluate is true.
* **eval_frequency** Time between measurements in seconds ros-time. Only available if evaluate is true. Default is 5.0.
* **ns_voxblox** Namespace of voxblox, including the node name. Only available if evaluate is true. Default is '/voxblox/voxblox_node'.

### Input Topics
* **ue_raw_in** of type `unreal_cv_ros.msg/UeSensorRaw`. Output of the unreal\_ros\_client for performance measurements. Only available if monitor is true.
* **ue_out_in** of type `sensor_msgs.msg/PointCloud2`. Output of the sensor model for performance measurements. Only available if monitor is true.

### Services
* **display_monitor** of type `std_srvs.srv/Empty`. Print the current monitoring measurements to console. Only available if monitor is true.


# Working with Unreal

## Unrealcv Plugin Setup
### Standard Plugin
For the modes `test` and `standard` the default unrealcv plugin is sufficient. Install it to the project or engine as suggested on [their website](http://docs.unrealcv.org/en/master/plugin/install.html). 

### Adding the 'vget /uecvros/full' command
This additional command is required to operate in`fast` mode.
* **Compiled Plugin** The compiled plugin can be downloaded from the [data repository](#Data-Repository).

* **Compile it yourself**  To compile the plugin, first create an unreal engine development environment as is explained [here](http://docs.unrealcv.org/en/master/plugin/develop.html). Then change the source code of the unrealcv plugin in your project (eg. `UnrealProjects/playground/Plugins/UnrealCV/Source/UnrealCV/Private/Commands`) to include the new command:
  - In `CameraHandler.h` add the command declaration
  - In `CameraHandler.cpp` add the command dispatcher and function body. 
  
These 3 code snippets can be copied from `unreal_cv_ros/content/CustomPluginCode.cpp`. Afterwards build the project as explained in the unrealcv docs. The compiled plugin can now be copied to other projects or the engine for use.

### Command reference 
This command take images and then request the passed position and orientation. This leaves time for the engine to finish rendering before the next images are requested.
* Syntax: `vget /uecvros/full X, Y, Z, p, y, r, collision, cameraID`. All argumentes, except the cameraID which is a uint, are floats. All arguments must be set. 
* Input: 
  - (X, Y, Z): New target position in unreal units.
  - (p, y, r): New target orientation in degrees (and unreal coordinate system).
  - collision: Set to 1 to check collision, -1 to ignore it.
  - cameraID:  ID of the camera to use in compliance with the unrealcv structure. Generally use 0.
* Output: 
  - In case of collision returns "Collision detected!" as unicode. 
  - Otherwise returns the stacked binary image data as string, where the first half representes the color image and the second half the depth image, both as npy arrays.

## Creating UE4 worlds
In order to easily create unreal_cv_ros compatible  UE4 worlds:
* Install and use unreal engine editor **4.16** for compatibility with unrealcv. (Note: Newer versions *should* work too but without guarantees).
* Make sure the unrealcv plugin is installed **and** activated in the current project (In the Editor check: Edit > Plugins > Science > Unreal CV, see [unrealcv docs](http://docs.unrealcv.org/en/master/plugin/install.html)).
* Set the player pawn to a flying spectator type with collision: World Settings > Game Mode > Selected GameMode > Default Pawn Class := UecvrosDefaultPawn. (If this is read-only just change to a custom GameMode.) 
  - Highly recommended: If using the `fast` mode, use the `UecvrosDefaultPawn`. The blueprint class can be downloaded from the [data repository](#Data-Repository).
  - Otherwise use the unreal engine built in `DefaultPawn`.
  
## When to use which mode
The unreal_cv_ros plugin is designed to work with all sorts of unreal worlds, however performance depends on how the world and the plugin is setup:

| Plugin | Unreal World | Method to use |
| --- | --- | --- |
| Custom | Custom | When the plugin and unreal world are modifiable, it is highly recommended to use the `fast` mode with the `UecvrosDefaultPawn`. This is not only the fastest combination, but it is also more stable and produces accurate and well-aligned pointclouds. |
| Custom | Static | If the unrealworld cannot be modified (usually it uses the `DefaultPawn`), it is still recommended to use the `fast` mode, since it runs considerably faster. However, the default camera may pose obstacles, such as motion blurr, that hinders alignment of the recorded images with the requested pose. Adjust the 'slowdown' parameter of the unreal_ros_client to give unreal enough time to properly process the desired images. |
| Static | Any | If the plugin cannot be modified, use the `standard` mode and the built in `DefaultPawn`. |
| `test` mode | Any | If you want to run the `test` mode, use the built in `DefaultPawn` to allow for suitable user control. |

## Pawns and Cameras 
Unrealcv works by capturing pawns and cameras in the unreal game, to which it attaches. Pawns are the physical entities that move around and collide with things, whereas cameras typically provide the views. Usually, pawns are not placed in the world, thus upon game start a Pawn will spawn at PlayerStart that is then possessed by player control and unrealcv.
* Cameras in unrealcv are stored in chronological order. If you want to access different cameras use the 'camera_id' param of the unreal_ros_client.
* The `DefaultPawn` is incoporates standard flying player control and a default view. However that incorporates all default features such as motion blurr when moving (or teleporting!) the pawn.
* The `UecvrosDefaultPawn` inherits from the `DefaultPawn`. Furthermore, it has a camera attached to it that can be modified. However this disables the default player camera orientation control (which is also the one accessed with the standard unrealcv plugin).

Custom camera settings can similarly be changed by editing the camera component. Notice that unrealcv overwrites certain camera settings (such as the resolution and field of view) using the unrealcv configuration file.

## Custom collision and camera settings
The default collision for the `DefaultPawn` and `UecvrosDefaultPawn` is a sphere of 35cm radius. For custom collision and camera, you need to create your own pawn blueprint (which inherits from the respective base class). E.g. an 'easy' way to create a custom `DefaultPawn` is as follows:
1. In the Modes window, search for 'DefaultPawn' and create an instance (drag and drop into the game world).
2. Select the pawn instance and click Blueprints > Convert selected actor to blueprint class...
3. Save the new class, e.g. in the content/blueprints folder as myDefaultPawn.
4. The class should now open in the blueprint editor, where its components can be edited.
5. To change the radius select the 'CollisionComponent', and under Details > Shape > SphereRadius := myValue. Notice that too short radii can allow the camera to enter certain objects, creating graphical artifacts.
6. Save the blueprint. Set the World Settings > Game Mode > Selected GameMode > Default Pawn Class := myDefaultPawn

## Static mesh collision
When creating UE4 worlds, it is worth double checking the player collision with static mesh actors (the scene). By default, unreal produces convex hulls as simple collision objects and checks against simple collision. However, this may result in faulty collision detection. Collision can be changed to match the visible mesh as follows (may degrade performance): 
1. Right click your object in the World Outliner and select "Edit 'your object'".
2. Set Collision > Collision Complexity := Use Complex Collision As Simple.
3. Save and close the editing window.

Current collision can be visualized by changing the viewmode in the unreal editor from "Lit" to "Player Collision". 

## Producing ground truth pointclouds
For simulation evaluation, ground truth meshes can be exported from the unreal editor and further processed using tools such as [CloudCompare](https://www.danielgm.net/cc/). A voxblox compatible ground truth pointcloud can be generated as follows:
* In the **Unreal Engine Editor**,
1. Select the objects of interes in the World Outliner
2. File > Export Selected...
3. Save as \*.obj file. (Currently no need to export the material too.)
  
* Open **CloudCompare**,
4. File > Import > my_mesh_export.obj
5. Use 'Edit > Multiply/Scale > 0.01' to compensate for unreal engine units (default is cm).
6. Use 'Edit > Apply transformation' to place and rotate the object relative to the PlayerStart settings from unreal. (I.e. with the origin at the PlayerStart location, x pointing towards the PlayerStart orientation, y-left and z-up).
7. Use 'Edit > Crop' to remove meshes outside the region of interest and unreachable points, such as the ground plane.
8. Click 'Sample points on a mesh' to create a pointcloud. (Currently no need to generate normals or color.)
9. 'File > Save' and save as a \*.ply in ASCII format
  
* With a **TextEditor**,
10. Open my_gt_pointcloud.ply and remove the "comment Author" and "obj_info" fields (lines 3 and 4, these create errors with pcl::plyreader).

## The Unreal Coordinate System
For application with the unreal\_ros\_client, the coordinate transformations are already implemented so no need to worry. For development/debugging tasks: Unreal and unrealcv use the following coordinate system: 

* **Unreal World** The default coordinate system is X-forward, Y-right, Z-up. Default units are cm.
* **Rotation Direction** Positive rotation directions around the unreal world coordinate axes are mathematically positive around the X axis and negative around the Y and Z axes.
* **Rotation parametrization** The Unreal engine interface and therefore also the unrealcv commands parse rotations as pitch-yaw-roll (pay attention to the order). However, inside the engine rotations are performed as Euler-XYZ rotations (i.e. roll-pitch-yaw). Default units are degrees.

# Examples
## Run in test mode
To illustrate the vision pipeline in stand-alone fashion, we run the unreal_ros_client in test mode with ground_truth as our sensor model. Please download the [RealisticRendering](http://docs.unrealcv.org/en/master/reference/model_zoo.html#rr) game binary and launch the game. In a command window type `roslaunch unreal_cv_ros example_test.launch` to start the pipeline and wait until the connection is setup (takes few seconds). You can now navigate the drone inside the game using the mouse and W-A-S-D keys while a rviz window displayes the produced ground truth pointclouds as well as the MAV pose in the unreal world frame.

**Note:** Since the taking of images and read-out of the unreal-pose is done sequentially, fast movement in the game may result in the frames not being well aligned.

## Run with MAV in standard mode
TODO: update this example to the current manager and simple trajectory publisher node.

(This example demonstrates the full scale MAV simulation using gazebo for MAV physics, a MPC high level and PID low level controller for trajectory tracking, voxblox for mapping and a planner node for trajectory generation. Setup and run the RealisticRendering demo as in the test example. In a command window type `roslaunch unreal_cv_ros example_full.launch` to start the pipeline. The simulation\_manager will supervise the startup of all elements (this may take few seconds). If everything sets up cleanly, the planner will start a random walk. In a rviz window, the current position of the MAV is depicted together with the executed and planned trajectories. Furthermore, the voxblox mesh representation of the room will be updated as it is explored.

**Note:** During simulation, the unreal game still takes commands from the user. Make sure to tab out of the game so that the user input does not interfere with the simulation setup. )


# Troubleshooting
Known Issues and what to do about them:
1. Error message "Error addressing the unrealcv client. Try restarting the game.":
    - Make sure that only a single unreal game **or** editor is running. When using the editor, the unrealcv plugin is loaded already during startup (without the game itself running!). Since unrealcv per default connects to `localhost:9000` the server will be blocked if any other instance of it is running.
2. The produced pointclouds are not well aligned with the requested pose / are smeared out:
    - If running in standard mode, consider switching to fast mode which is not only faster but also more stable. (In standard mode  every orientation change is carried out as rotation movement inducing increased rendering time and rotation offsets, especially for high rotation rates).
    - Give unreal engine more rendering time by adjusting the `slowdown` paramter of the unreal\_ros\_client. (This problem typically occurs for low rendering/frame rates in the unreal game).
3. I get collision warnings for quickly executed trajectories:
    - The unreal teleport action with collision checking sweeps (as far as I know) a linear path between every request. Try increasing the unreal\_ros\_client's update rate or slowing down simulated time.
