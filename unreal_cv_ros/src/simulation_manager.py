#!/usr/bin/env python

# ros
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Pose, Twist, Transform
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelState
from unreal_cv_ros.msg import UeSensorRaw
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty, SetBool
from gazebo_msgs.srv import SetModelState, GetModelState
from voxblox_msgs.srv import FilePath
import tf

# Python
import sys
import math
import numpy as np
import time
from collections import deque
import os
import datetime
import csv
import subprocess

# Plotting
from matplotlib import pyplot as plt


class SimulationManager:

    def __init__(self):
        '''  Initialize ros node and read params '''
        # Parse parameters
        self.ns_gazebo = rospy.get_param('~ns_gazebo', "/gazebo")   
        self.ns_mav = rospy.get_param('~ns_mav', "/firefly")
        self.ns_planner = rospy.get_param('~ns_planner', "/firefly/planner_node")
        self.planner_delay = rospy.get_param('~delay', 0.0)     # Waiting time until the planner is launched
        self.regulate = rospy.get_param('~regulate', False)     # Manage odom throughput for unreal_ros_client
        self.monitor = rospy.get_param('~monitor', False)       # Measure performance of unreal pipeline
        self.evaluate = rospy.get_param('~evaluate', False)     # Periodically save the voxblox state

        if self.monitor:
            self.horizon = rospy.get_param('~horizon', 10)  # How many messages are kept for monitoring

            # Monitoring arrays
            self.mon_client_rate_real = deque(maxlen=self.horizon)
            self.mon_client_rate_ros = deque(maxlen=self.horizon)
            self.mon_client_prev_real = None
            self.mon_client_prev_ros = None
            self.mon_client_delay_ros = deque(maxlen=self.horizon)
            self.mon_client_time = deque(maxlen=self.horizon)
            self.mon_sensor_rate_real = deque(maxlen=self.horizon)
            self.mon_sensor_rate_ros = deque(maxlen=self.horizon)
            self.mon_sensor_prev_real = None
            self.mon_sensor_prev_ros = None
            self.mon_sensor_delay_ros = deque(maxlen=self.horizon)
            self.mon_sensor_time = deque(maxlen=self.horizon)

            # Subscribers
            self.ue_raw_sub = rospy.Subscriber("ue_raw_in", UeSensorRaw, self.mon_raw_callback, queue_size=10)

            # Printing service
            rospy.Service('~display_monitor', Empty, self.mon_print_handle)

        if self.monitor or self.evaluate:
            self.ue_out_sub = rospy.Subscriber("ue_out_in", PointCloud2, self.mon_out_callback, queue_size=10)

        if self.evaluate:
            # Setup parameters
            self.eval_directory = rospy.get_param('~eval_directory', 'DirParamNotSet')  # Periodically save voxblox map
            if not os.path.isdir(self.eval_directory):
                rospy.logfatal("Invalid target directory '%s'.", self.eval_directory)
                sys.exit(-1)

            self.ns_voxblox = rospy.get_param('~ns_voxblox', "/voxblox/voxblox_node")
            self.eval_frequency = rospy.get_param('~eval_frequency', 5.0)  # Save rate in seconds

            # Statistics
            self.eval_walltime_0 = None
            self.eval_rostime_0 = None
            self.eval_n_maps = 0
            self.eval_n_pointclouds = 0

            # Setup data directory
            self.eval_directory = os.path.join(self.eval_directory,
                                               datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
            os.mkdir(self.eval_directory)
            os.mkdir(os.path.join(self.eval_directory, "voxblox_maps"))
            self.eval_data_file = open(os.path.join(self.eval_directory, "voxblox_data.csv"), 'wb')
            self.eval_writer = csv.writer(self.eval_data_file, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            self.eval_writer.writerow(['MapName', 'RosTime', 'WallTime', 'NPointclouds'])
            self.eval_writer.writerow(['Unit', 'seconds', 'seconds', '-'])
            self.eval_log_file = open(os.path.join(self.eval_directory, "data_log.txt"), 'a')

            # Finish
            self.writelog("Data folder created at '%s'." % self.eval_directory)
            rospy.loginfo("Data folder created at '%s'." % self.eval_directory)
            self.eval_voxblox_service = rospy.ServiceProxy(self.ns_voxblox + "/save_map", FilePath)
            rospy.on_shutdown(self.eval_finish)

        if self.regulate:
            # Subscribers and publishers
            self.odom_sub = rospy.Subscriber("odom_in", Odometry, self.odom_callback, queue_size=10)
            self.odom_pub = rospy.Publisher("odom_out", Odometry, queue_size=10)

        # Run the startup
        self.launch_simulation()

    def launch_simulation(self):
        # Wait for Gazebo services
        rospy.loginfo("Starting simulation setup coordination...")
        rospy.wait_for_service(self.ns_gazebo + "/unpause_physics")
        rospy.wait_for_service(self.ns_gazebo + "/set_model_state")

        # Prepare initialization trajectory command
        traj_pub = rospy.Publisher(self.ns_mav + "/command/trajectory", MultiDOFJointTrajectory, queue_size=10)
        traj_msg = MultiDOFJointTrajectory()
        traj_msg.joint_names = ["base_link"]
        transforms = Transform()
        transforms.translation.x = 0.0
        transforms.translation.y = 0.0
        transforms.translation.z = 0.0
        point = MultiDOFJointTrajectoryPoint([transforms], [Twist()], [Twist()], rospy.Duration(0))
        traj_msg.points.append(point)

        # Prepare initialization Get/SetModelState service
        set_model_srv = rospy.ServiceProxy(self.ns_gazebo + "/set_model_state", SetModelState)
        get_model_srv = rospy.ServiceProxy(self.ns_gazebo + "/get_model_state", GetModelState)
        mav_name = self.ns_mav[np.max([i for i in range(len(self.ns_mav)) if self.ns_mav[i] == "/"])+1:]
        model_state_set = ModelState(mav_name, Pose(), Twist(), "world")

        # Wake up gazebo
        rospy.loginfo("Waiting for gazebo to wake up ...")
        unpause_srv = rospy.ServiceProxy(self.ns_gazebo + "/unpause_physics", Empty)
        while not unpause_srv():
            rospy.sleep(0.1)
        rospy.loginfo("Waiting for gazebo to wake up ... done.")

        # Wait for drone to spawn (imu is publishing)
        rospy.loginfo("Waiting for MAV to spawn ...")
        rospy.wait_for_message(self.ns_mav + "/imu", Imu)

        # Initialize drone stable at [0, 0, 0]
        dist = 10       # Position and velocity
        while dist >= 0.1:
            traj_msg.header.stamp = rospy.Time.now()
            traj_pub.publish(traj_msg)
            set_model_srv(model_state_set)
            rospy.sleep(0.1)
            state = get_model_srv(mav_name, "world")
            pos = state.pose.position
            twist = state.twist.linear
            dist = np.sqrt(pos.x**2 + pos.y**2 + pos.z**2) + np.sqrt(twist.x**2 + twist.y**2 + twist.z**2)
        rospy.loginfo("Waiting for MAV to spawn ... done.")

        # Wait for unreal client
        rospy.loginfo("Waiting for unreal client to setup ...")
        rospy.wait_for_message("ue_sensor_raw", UeSensorRaw)
        rospy.loginfo("Waiting for unreal client to setup ... done.")

        # Launch planner (by service, every planner needs to advertise this service when ready)
        rospy.loginfo("Waiting for planner to be ready...")
        rospy.wait_for_service(self.ns_planner + "/toggle_running")

        if self.planner_delay > 0:
            rospy.loginfo("Waiting for planner to be ready... done. Launch in %d seconds.", self.planner_delay)
            rospy.sleep(self.planner_delay)
        else:
            rospy.loginfo("Waiting for planner to be ready... done.")
        run_planner_srv = rospy.ServiceProxy(self.ns_planner + "/toggle_running", SetBool)
        run_planner_srv(True)

        # Finish
        rospy.loginfo("Succesfully started the simulation!")
        if self.evaluate:
            self.writelog("Succesfully started the simulation.")
            self.eval_setup()

    def eval_setup(self):
        # Dump complete rosparams for reference
        subprocess.check_call(["rosparam", "dump", os.path.join(self.eval_directory, "rosparams.yaml"), "/"])
        self.writelog("Dumped the parameter server into 'rosparams.yaml'.")

        # Setup first measurements
        self.eval_walltime_0 = time.time()
        self.eval_rostime_0 = rospy.get_time()
        self.eval_n_maps = 0
        self.eval_n_pointclouds = 1

        # Periodic evaluation (call once for initial measurement)
        self.eval_callback(None)
        rospy.Timer(rospy.Duration(self.eval_frequency), self.eval_callback)

    def eval_callback(self, _):
        # Produce a data point
        time_real = time.time() - self.eval_walltime_0
        time_ros = rospy.get_time() - self.eval_rostime_0
        map_name = "{0:05d}".format(self.eval_n_maps)
        self.eval_writer.writerow([map_name, time_ros, time_real, self.eval_n_pointclouds])
        self.eval_voxblox_service(os.path.join(self.eval_directory, "voxblox_maps", map_name + ".vxblx"))
        self.eval_n_pointclouds = 0
        self.eval_n_maps += 1

    def eval_finish(self):
        self.eval_data_file.close()
        map_path = os.path.join(self.eval_directory, "voxblox_maps")
        n_maps = len([f for f in os.listdir(map_path) if os.path.isfile(os.path.join(map_path, f))])
        self.writelog("Finished the simulation, %d/%d maps created." % (n_maps, self.eval_n_maps))
        self.eval_log_file.close()
        rospy.loginfo("On simulation_manager shutdown: closing data files.")

    def writelog(self, text):
        # In case of simulation data being stored, maintain a log file
        if not self.evaluate:
            return
        self.eval_log_file.write(datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S] ") + text + "\n")

    def mon_raw_callback(self, ros_data):
        # Measure walltime and rostime between callbacks
        time_real = time.time()
        time_ros = rospy.get_time()
        if self.mon_client_prev_real is None:
            # Initialization
            self.mon_client_prev_real = time_real
            self.mon_client_prev_ros = time_ros
            return

        self.mon_client_rate_real.append(time_real-self.mon_client_prev_real)
        self.mon_client_prev_real = time_real
        self.mon_client_rate_ros.append(time_ros-self.mon_client_prev_ros)
        self.mon_client_prev_ros = time_ros
        self.mon_client_delay_ros.append(time_ros - ros_data.header.stamp.to_sec())
        self.mon_client_time.append(time_ros)

    def mon_out_callback(self, ros_data):
        if self.evaluate:
            self.eval_n_pointclouds += 1

        if self.monitor:
            # Measure walltime and rostime between callbacks
            time_real = time.time()
            time_ros = rospy.get_time()
            if self.mon_sensor_prev_real is None:
                # Initialization
                self.mon_sensor_prev_real = time_real
                self.mon_sensor_prev_ros = time_ros
                return

            self.mon_sensor_rate_real.append(time_real - self.mon_sensor_prev_real)
            self.mon_sensor_prev_real = time_real
            self.mon_sensor_rate_ros.append(time_ros - self.mon_sensor_prev_ros)
            self.mon_sensor_prev_ros = time_ros
            self.mon_sensor_delay_ros.append(time_ros - ros_data.header.stamp.to_sec())
            self.mon_sensor_time.append(time_ros)

    def mon_print_handle(self, _):
        print("="*14 + " performance monitor " + "="*14)
        print("Field [s]:           avg  -  std  -  min  -  max ")
        values = np.array(self.mon_client_rate_real)
        if len(values) > 0:
            print("Client rate (real): {0:05.2f} - {1:05.2f} - {2:05.2f} - {3:05.2f}"
                  .format(np.mean(values), np.std(values), np.min(values), np.max(values)))
        values = np.array(self.mon_client_rate_ros)
        if len(values) > 0:
            print("Client rate (ros):  {0:05.2f} - {1:05.2f} - {2:05.2f} - {3:05.2f}"
                  .format(np.mean(values), np.std(values), np.min(values), np.max(values)))
        values = np.array(self.mon_client_delay_ros)
        if len(values) > 0:
            print("Client delay (ros): {0:05.2f} - {1:05.2f} - {2:05.2f} - {3:05.2f}"
                  .format(np.mean(values), np.std(values), np.min(values), np.max(values)))
        values = np.array(self.mon_sensor_rate_real)
        if len(values) > 0:
            print("Sensor rate (real): {0:05.2f} - {1:05.2f} - {2:05.2f} - {3:05.2f}"
                  .format(np.mean(values), np.std(values), np.min(values), np.max(values)))
        values = np.array(self.mon_sensor_rate_ros)
        if len(values) > 0:
            print("Sensor rate (ros):  {0:05.2f} - {1:05.2f} - {2:05.2f} - {3:05.2f}"
                  .format(np.mean(values), np.std(values), np.min(values), np.max(values)))
        values = np.array(self.mon_sensor_delay_ros)
        if len(values) > 0:
            print("Sensor delay (ros): {0:05.2f} - {1:05.2f} - {2:05.2f} - {3:05.2f}"
                  .format(np.mean(values), np.std(values), np.min(values), np.max(values)))
        print("=" * 49)


if __name__ == '__main__':
    rospy.init_node('simulation_manager', anonymous=True)
    sm = SimulationManager()
    rospy.spin()

