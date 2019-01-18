unreal_cv_ros is a package to allow ROS based simulation of a MAV equipped with a 3D-reconstruction sensor. The simulation is performed inside an [unreal engine](https://www.unrealengine.com/en-US/what-is-unreal-engine-4) game. The node-game communcation is carried out utilizing the [unrealcv](https://github.com/unrealcv/unrealcv) computer vision plugin for unreal engine 4 (UE4).

# Table of Contents
**ROS nodes**
* [unreal_ros_client](#unreal_ros_client)
* [sensor_model](#sensor_model)
* [simulation_manager](#simulation_manager)

**Setting up Unreal**
* [Unrealcv Plugin Setup](#Unrealcv-Plugin-Setup)
* [Creating UE4 worlds](#Creating-UE4-worlds)
* [Custom collision radius](#Custom-collision-radius)
* [The Unreal Coordinate System](#The-Unreal-Coordinate-System)

**Examples**
* [Run in test mode](#Run-in-test-mode)
* [Run in standard mode](#Run-in-standard-mode)

**Troubleshooting**
* [Troubleshooting](#Troubleshooting)

## Dependencies
What should all be added here? The unreal_ros_client node depends on the unrealcv python library `pip install unrealcv`.

# ROS nodes
## unreal_ros_client
This node manages the unrealcv client and the connection with a running UE4 game. It sets the MAV position and orientation within the unreal game and produces the images and camera calibration used by a 3D sensor model.

### Parameters
* **mode** In which mode the client is operated. Currently implemented are:
  * **test** Navigate the MAV manually in the unreal game. The client will periodically publish the sensor data.
  * **standard** The camera pose is set based on the `/odometry` topic and images are taken. Uses the default unrealcv plugin, operates at ~1 Hz.
  * **fast** Similar to standard, but requires a custom unrealcv command (See [Unrealcv Plugin Setup](#Unrealcv-Plugin-Setup)).  Operates at ~4 Hz.
  
  Default is 'standard'.
* **collision_on** Set to true to check for collision in the unreal game. Set to false to set the camera anyway. May result in rendering artifacts if the camera overlaps with objects. Default is true.
* **collision_tol** Distance threshold for collision detection in unreal units (default is cm). Will trigger a collision warning if the requested and realized position are further away than the threshold. Default is 10.
* **Hint:** To change the resolution and field of view (FOV) of the camera, the [unrealcv configuration file](http://docs.unrealcv.org/en/master/plugin/config.html) needs to be changed. Its path is displayed when the unreal_ros_client node is launched.

### Input Topics
* **odometry** of type `nav_msgs.msg/Odometry`. Set the camera pose w.r.t. its position and yaw at the connection of the client. Only appears if test is false.

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
* **ns_planner** Namespace of the planner, including the node name. Default is '/firefly/random_planner'.
* **monitor** Set to true to measure the unreal vision pipeline's performance. Default is false.
* **horizon** How many datapoints are kept in the performance measurement. Default is 10.

### Input Topics
* **ue_raw_in** of type `unreal_cv_ros.msg/UeSensorRaw`. Output of the unreal\_ros\_client for performance measurements. Only available if monitor is true.
* **ue_out_in** of type `sensor_msgs.msg/PointCloud2`. Output of the sensor model for performance measurements. Only available if monitor is true.

### Services
* **display_monitor** of type `std_srvs.srv/Empty`. Print the current monitoring measurements to console.


# Setting up Unreal

## Unrealcv Plugin Setup
### Standard Plugin
For the modes 'test' and 'standard' the default unrealcv plugin is sufficient. Install it to the project or engine as suggested on their [website](http://docs.unrealcv.org/en/master/plugin/install.html). This repo was tested on unrealcv v0.3.10.

### Adding the 'vget /uecvros/full' command
This additional command is needed for the 'fast' mode.
* **Compiled Plugin** TODO: We can make this available somewhere I think.

* **Compile it yourself**  Create an unreal engine development environment as is explained [here](http://docs.unrealcv.org/en/master/plugin/develop.html). Change the source code of the unrealcv plugin in the project (eg. `UnrealProjects/playground/Plugins/UnrealCV/Source/UnrealCV/Private/Commands`) to include the new command:
In `CameraHandler.h` add the declaration:
```c++
/** Run a step of the uecvros routine */
FExecStatus UecvrosFull(const TArray<FString>& Args);
```
  In `CameraHandler.cpp` add the command dispatcher and function body (Copy paste the code from content/CommandCode.cpp). Afterwards build the project as explained in the unrealcv docs. The compiled plugin can thereafter be copied to other projects or the engine.

* **command reference** Take images and then request the next position. This leaves time for the engine to finish movement before the next images are requested.
  - Syntax: `vget /uecvros/full X, Y, Z, p, y, r, tol, X_old, Y_old, Z_old`. All argumentes are floats and need to be set. 
  - Input: 
    - (X, Y, Z): New target position in unreal units.
    - (p, y, r): New target orientation in degrees (and unreal coordinate system).
    - tol: Distance threshold for collision detection. Set to -1 to turn collision off.
    - (X, Y, Z)\_old: Previous target position against which collision is tested.
  - Output: 
    - In case of collision returns a Collision warning. 
    - Otherwise returns binary image data, where the first half representes the color image and the second half the depth image, both as npy arrays.

## Creating UE4 worlds
In order to easily create unreal_cv_ros compatible worlds UE4 worlds:
* Install and use unreal engine editor **4.16** for compatibility with unrealcv. (Note: Newer versions *should* work too but without guarantees).
* Make sure the unrealcv plugin is installed **and** activated in the current project (In the Editor check: Edit > Plugins > Science > Unreal CV, see [unrealcv docs](http://docs.unrealcv.org/en/master/plugin/install.html)).
* Set the player to a spectator type with collision: World Settings > Game Mode > Selected GameMode > Default Pawn Class := DefaultPawn. (If this is read-only just change toa custom gamemode above.)

## Custom collision radius
The default collision for the *DefaultPawn* is a sphere of radius 35cm. For custom collision (radius), you need to create your own pawn blueprint (with DefaultPawn as base class). 'Easy' way to create a pawn of custom collision radius:
1. In the Modes window, search for 'DefaultPawn' and create an instance (drag and drop into the game world).
2. Select the pawn instance and click Blueprints > Convert selected actor to blueprint class...
3. Save the new class, e.g. in the content/blueprints folder as myDefaultPawn.
4. The class should now open in the blueprint editor, where its components can be edited.
5. To change the radius elect the 'CollisionComponent', and under Details > Shape > SphereRadius := myValue. Notice that too short radii can allow the camera to enter certain objects, creating graphical artifacts.
6. Save the blueprint. Set the World Settings > Game Mode > Selected GameMode > Default Pawn Class := myDefaultPawn

## The Unreal Coordinate System
For application with the unreal\_ros\_client, the coordinate transformations are already implemented so no need to worry. For devel/debug tasks: Unreal and unrealv use the following coordinate system: 

* **Unreal World** The default coordsystem is X-forward, Y-left, Z-up. Default units are cm.
* **Rotation Direction** Positive rotation directions around the unreal world coordinate axes are mathematically positive around the X-axis and negative around the Y and Z axes.
* **Rotation parametrization** The Unreal engine interface and therefore also the unrealcv commands parse rotations as pitch-yaw-roll (pay attention to the order). However, inside the engine rotations are performed as Euler-XYZ rotations (i.e. roll-pitch-yaw). Default units are degrees.

# Examples
## Run in test mode
To illustrate the vision pipeline in stand-alone fashion, we run the unreal_ros_client in test mode with ground_truth as our sensor model. Please download the [RealisticRendering](http://docs.unrealcv.org/en/master/reference/model_zoo.html#rr) game binary and launch the game. In a command window type `roslaunch unreal_cv_ros example_test.launch` to start the pipeline and wait until the connection is setup (takes few seconds). You can now navigate the drone inside the game using the mouse and W-A-S-D keys while a rviz window displayes the produced ground truth pointclouds as well as the MAV pose in the unreal world frame.

**Note:** Since the taking of images and read-out of the unreal-pose is done sequentially, fast movement in the game may result in the frames not being well aligned.

## Run in standard mode
This example demonstrates the full simulation using gazebo, an MPC high level and PID low level controller, voxblox and a planner node. Setup and run the RealisticRendering demo as in the test example. In a command window type `roslaunch unreal_cv_ros example_full.launch` to start the pipeline. The simulation\_manager will supervise the startup of all elements (this may take few seconds). If everything sets up cleanly, the RandomPlanner will start a random walk while remaining collision free. In a rviz window, the current position of the MAV is depicted together with the current planned trajectory. Furthermore, the voxblox mesh representation of the room will be updated as it is explored.

**Note:** During simulation, the unreal game still takes commands from the user. Make sure to tab out of the game so that the user input does not interferewith the simulation setup. 

**More Notes:** The RandomPlanner is optimistic and will treat unseen areas as collision free. It may therefore collide in such areas.

# Troubleshooting
1. Error addressing the unrealcv client. Try restarting the game.
   - Make sure that only a single unreal game **or** editor is running. When using the editor, the unrealcv plugin is loaded already during startup (without the game itself running!). Since unrealcv per default connects to `localhost:9000` the server will be blocked if any other instance of it is running.
