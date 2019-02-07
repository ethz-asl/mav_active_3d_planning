unreal_cv_ros is a package to allow ROS based simulation of a MAV equipped with a 3D-reconstruction sensor. The simulation is performed inside an [unreal engine 4](https://www.unrealengine.com/en-US/what-is-unreal-engine-4) game. The node-game communcation is carried out utilizing the [unrealcv](https://github.com/unrealcv/unrealcv) computer vision plugin for unreal engine 4 (UE4).

# Table of Contents
**Installation**
* [Installation](#Installation)
* [Dependencies](#Dependencies)

**ROS nodes**
* [unreal_ros_client](#unreal_ros_client)
* [sensor_model](#sensor_model)
* [simulation_manager](#simulation_manager)

**Setting up Unreal**
* [Unrealcv Plugin Setup](#Unrealcv-Plugin-Setup)
* [Creating UE4 worlds](#Creating-UE4-worlds)
* [Static mesh collision](#Static-mesh-collision)
* [Custom collision radius](#Custom-collision-radius)
* [The Unreal Coordinate System](#The-Unreal-Coordinate-System)

**Examples**
* [Run in test mode](#Run-in-test-mode)
* [Run in standard mode](#Run-in-standard-mode)

**Troubleshooting**
* [Frequent Issues](#Troubleshooting)

# Installation
Install as every other ROS package...

## Dependencies
The unreal_ros_client node depends on the unrealcv python library `pip install unrealcv`.

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
* **publish_tf** If true, the client pulishes a tf-transform of the camera pose for every taken image with matching timestamps. Default is False.
* **slowdown** Artificially slows down the time between setting the pose and taking images to give unreal engine more time to render the new view. Slowdown is expected as wait duration in seconds wall-time. Default is 0.0.
* **Hint:** To change the resolution and field of view (FOV) of the camera, the [unrealcv configuration file](http://docs.unrealcv.org/en/master/plugin/config.html) needs to be changed. Its path is displayed when the unreal_ros_client node is launched.

### Input Topics
* **odometry** of type `nav_msgs.msg/Odometry`. Set the camera pose w.r.t. its position and yaw at the connection of the client. Does not appear in `test` mode.

### Output Topics
* **ue_sensor_raw** of type `unreal_cv_ros.msg/UeSensorRaw`. The output of the in-game image capture, containing a color and a depth image encoded as npy binaries.


## sensor_model
This node converts the UE client output into a pointcloud for further processing and artificially simulates the behaviour of a 3D sensor (e.g. a stereo reconstruction pipeline).

### Parameters
* **model_type** Which sensor to simulate. Currently implemented are: 
  * **ground_truth:** Produces the ground truth pointcloud without additional noise. 
  
  Default is 'ground_truth'.

### Input Topics
* **ue_sensor_raw** of type `unreal_cv_ros.msg/UeSensorRaw`. Output of the unreal\_ros\_client that is to be further processed.

### Output Topics
* **ue_sensor_out** of type `sensor_msgs.msg/PointCloud2`. Resulting pointcloud after applying the simulated sensing pipeline.


## simulation_manager
This node is used to coordinate the full MAV simulation using gazebo as a physics engine and an unreal game for perception and collision modeling. It is used to coordinate simulation setup and monitor or supervise the unreal_ros vision pipeline.

### Parameters
* **ns_gazebo** Namespace of gazebo, including the node name. Default is '/gazebo'.
* **ns_mav** Namespace of the MAV, which is expected to end with the MAV name. Default is '/firefly'.
* **ns_planner** Namespace of the planner, including the node name. Default is '/firefly/planner_node'.
* **monitor** Set to true to measure the unreal vision pipeline's performance. Default is False.
* **horizon** How many datapoints are kept in the performance measurement. Default is 10.
* **delay** Additional waiting time in seconds before launching the planner to give everything time to settle. Default is 0.0.

### Input Topics
* **ue_raw_in** of type `unreal_cv_ros.msg/UeSensorRaw`. Output of the unreal\_ros\_client for performance measurements. Only available if monitor is true.
* **ue_out_in** of type `sensor_msgs.msg/PointCloud2`. Output of the sensor model for performance measurements. Only available if monitor is true.

### Services
* **display_monitor** of type `std_srvs.srv/Empty`. Print the current monitoring measurements to console. Only available if monitor is true.


# Setting up Unreal

## Unrealcv Plugin Setup
### Standard Plugin
For the modes `test` and `standard` the default unrealcv plugin is sufficient. Install it to the project or engine as suggested on their [website](http://docs.unrealcv.org/en/master/plugin/install.html). This repo was tested on unrealcv v0.3.10.

### Adding the 'vget /uecvros/full' command
This additional command is required to operate in`fast` mode.
* **Compiled Plugin** TODO: We can make this available somewhere I think.

* **Compile it yourself**  To compile the plugin, first create an unreal engine development environment as is explained [here](http://docs.unrealcv.org/en/master/plugin/develop.html). Then change the source code of the unrealcv plugin in your project (eg. `UnrealProjects/playground/Plugins/UnrealCV/Source/UnrealCV/Private/Commands`) to include the new command:
  - In `CameraHandler.h` add the command declaration
  - In `CameraHandler.cpp` add the command dispatcher and function body. 
These 3 code snippets can be copied from `unreal_cv_ros/content/CustomPluginCode.cpp`. Afterwards build the project as explained in the unrealcv docs. The compiled plugin can now be copied to other projects or the engine for use.

### Command reference 
This command take images and then request the passed position and orientation. This leaves time for the engine to finish rendering before the next images are requested.
* Syntax: `vget /uecvros/full X, Y, Z, p, y, r, collision`. All argumentes are floats and need to be set. 
* Input: 
  - (X, Y, Z): New target position in unreal units.
  - (p, y, r): New target orientation in degrees (and unreal coordinate system).
  - collision: Set to 1 to check collision, -1 to ignore it.
* Output: 
  - In case of collision returns "Collision detected!" as unicode. 
  - Otherwise returns the stacked binary image data as string, where the first half representes the color image and the second half the depth image, both as npy arrays.

## Creating UE4 worlds
In order to easily create unreal_cv_ros compatible  UE4 worlds:
* Install and use unreal engine editor **4.16** for compatibility with unrealcv. (Note: Newer versions *should* work too but without guarantees).
* Make sure the unrealcv plugin is installed **and** activated in the current project (In the Editor check: Edit > Plugins > Science > Unreal CV, see [unrealcv docs](http://docs.unrealcv.org/en/master/plugin/install.html)).
* Set the player pawn to DefaultPawn, a flying spectator type with collision: World Settings > Game Mode > Selected GameMode > Default Pawn Class := DefaultPawn. (If this is read-only just change toa custom gamemode above.)

## Static mesh collision
When creating UE4 worlds, it is worth double checking the player collision with static mesh actors (the scene). By default, unreal produces convex hulls as collision objects. However, this may result in faulty collision detection. Collision can be changed to match the visible mesh as follows (may degrade performance): 
1. Right click your object in the World Outliner and select "Edit 'your object'".
2. Set Collision > Collision Complexity := Use Complex Collision As Simple.
3. Save and close the editing window.

Current collision can be visualized by changing the viewmode in the unreal editor from "Lit" to "Player Collision". 

## Custom collision radius
The default collision for the 'DefaultPawn' is a sphere of radius 35cm. For custom collision radii, you need to create your own pawn blueprint (with DefaultPawn as base class). An 'easy' way to create a pawn of custom collision radiusis as follows:
1. In the Modes window, search for 'DefaultPawn' and create an instance (drag and drop into the game world).
2. Select the pawn instance and click Blueprints > Convert selected actor to blueprint class...
3. Save the new class, e.g. in the content/blueprints folder as myDefaultPawn.
4. The class should now open in the blueprint editor, where its components can be edited.
5. To change the radius elect the 'CollisionComponent', and under Details > Shape > SphereRadius := myValue. Notice that too short radii can allow the camera to enter certain objects, creating graphical artifacts.
6. Save the blueprint. Set the World Settings > Game Mode > Selected GameMode > Default Pawn Class := myDefaultPawn

## The Unreal Coordinate System
For application with the unreal\_ros\_client, the coordinate transformations are already implemented so no need to worry. For development/debugging tasks: Unreal and unrealcv use the following coordinate system: 

* **Unreal World** The default coordinate system is X-forward, Y-right, Z-up. Default units are cm.
* **Rotation Direction** Positive rotation directions around the unreal world coordinate axes are mathematically positive around the X axis and negative around the Y and Z axes.
* **Rotation parametrization** The Unreal engine interface and therefore also the unrealcv commands parse rotations as pitch-yaw-roll (pay attention to the order). However, inside the engine rotations are performed as Euler-XYZ rotations (i.e. roll-pitch-yaw). Default units are degrees.

# Examples
## Run in test mode
To illustrate the vision pipeline in stand-alone fashion, we run the unreal_ros_client in test mode with ground_truth as our sensor model. Please download the [RealisticRendering](http://docs.unrealcv.org/en/master/reference/model_zoo.html#rr) game binary and launch the game. In a command window type `roslaunch unreal_cv_ros example_test.launch` to start the pipeline and wait until the connection is setup (takes few seconds). You can now navigate the drone inside the game using the mouse and W-A-S-D keys while a rviz window displayes the produced ground truth pointclouds as well as the MAV pose in the unreal world frame.

**Note:** Since the taking of images and read-out of the unreal-pose is done sequentially, fast movement in the game may result in the frames not being well aligned.

## Run in standard mode
TODO: update this example with a finished mav\_active\_3d\_planning node.

This example demonstrates the full scale MAV simulation using gazebo for MAV physics, a MPC high level and PID low level controller for trajectory tracking, voxblox for mapping and a planner node for trajectory generation. Setup and run the RealisticRendering demo as in the test example. In a command window type `roslaunch unreal_cv_ros example_full.launch` to start the pipeline. The simulation\_manager will supervise the startup of all elements (this may take few seconds). If everything sets up cleanly, the planner will start a random walk. In a rviz window, the current position of the MAV is depicted together with the executed and planned trajectories. Furthermore, the voxblox mesh representation of the room will be updated as it is explored.

**Note:** During simulation, the unreal game still takes commands from the user. Make sure to tab out of the game so that the user input does not interfere with the simulation setup. 


# Troubleshooting
Known Issues and what to do about them:
1. Error message "Error addressing the unrealcv client. Try restarting the game.":
    - Make sure that only a single unreal game **or** editor is running. When using the editor, the unrealcv plugin is loaded already during startup (without the game itself running!). Since unrealcv per default connects to `localhost:9000` the server will be blocked if any other instance of it is running.
2. The produced pointclouds are not well aligned with the requested pose / are smeared out:
    - If running in standard mode, consider switching to fast mode which is not only faster but also more stable. (In standard mode  every orientation change is carried out as rotation movement inducing increased rendering time and rotation offsets, especially for high rotation rates).
    - Give unreal engine more rendering time by adjusting the `slowdown` paramter of the unreal\_ros\_client. (This problem typically occurs for low rendering/frame rates in the unreal game).
3. I get collision warnings for quickly executed trajectories:
    - The unreal teleport action with collision checking sweeps (as far as I know) a linear path between every request. Try increasing the unreal\_ros\_client's update rate or slowing down simulated time.
