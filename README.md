unreal_cv_ros is a package to allow ROS based simulation of a MAV equipped with a 3D-reconstruction sensor. The simulation is performed inside an [unreal engine](https://www.unrealengine.com/en-US/what-is-unreal-engine-4) game. The node-game communcation is carried out utilizing the [unrealcv](https://github.com/unrealcv/unrealcv) computer vision plugin for unreal engine 4 (UE4).

# Table of Contents
* [unreal_ros_client](#unreal_ros_client)
* [sensor_model](#sensor_model)
* [Example](#Example)

## Dependencies
What should all be added here? The unreal_ros_client node depends on the unrealcv python library `pip install unrealcv`.

# unreal_ros_client
This node manages the unrealcv client and the connection with a running UE4 game. It sets the MAV position and orientation within the unreal game and produces the images and camera calibration used by a 3D sensor model.

## Parameters
* **test** Set to true to manually navigate the camera inside the UE4 game and use the client to broadcast its pose (with respect to the world frame, located at the origin of the unreal coordinate system). If false, the MAVs pose is set by the odometry topic (in odometry frame, with the origin at the camera position at the point of connection). Default is false.
* **collision_on** Set to true to check for collision in the unreal game. Set to false to set the camera anyway. May result in rendering artifacts if the camera overlaps with objects. Default is true.
* **collision_tol** Distance threshold for collision detection in unreal units (default is cm). Will trigger a collision warning if the requested and realized position are further away than the threshold. Default is 10.
* **Hint:** To change the resolution and field of view (FOV) of the camera, the [unrealcv configuration file](http://docs.unrealcv.org/en/master/plugin/config.html) needs to be changed. Its path is displayed when launching the unreal_ros_client node.

## Input Topics
* **odometry** of type `nav_msgs.msg/Odometry`. Set the mav pose w.r.t. its positition at the connection of the client. Only appears if test is false.

## Output Topics
* **ue_sensor_raw** of type `unreal_cv_ros.msg/UeSensorRaw`. The output of the in-game image capture, containing a color and a depth image as well as a camera_info with the camera intrinsics.


# sensor_model
This node converts the UE game output into a pointcloud for further processing and artificially simulates the behaviour of a 3D sensor (e.g. a stereo reconstruction pipeline).

## Parameters
* **model_type** Which sensor to simulate. Currently implemented are: **ground_truth:** Produces the ground truth pointcloud without additional noise. Default is ground_truth.

## Input Topics
* **ue_sensor_raw** of type `unreal_cv_ros.msg/UeSensorRaw`. Output of the unreal_ros_client that is to be further processed.

## Output Topics
* **ue_sensor_out** of type `sensor_msgs.msg/PointCloud2`. Result after applying the simulated sensing pipeline.


# Example
To illustrate the pipeline we run the unreal_ros_client in test mode with ground_truth as our sensor model. Please download the [RealisticRendering](http://docs.unrealcv.org/en/master/reference/model_zoo.html#rr) game binary and launch the game. In a command window type `roslaunch unreal_cv_ros test_pointcloud.launch` to start the pipeline and wait until the connection is setup (takes few seconds). You can now navigate the drone inside the game using the mouse and W-A-S-D keys while a rviz window displayes the produced ground truth pointclouds as well as the MAV pose in the unreal world frame.
