#!/usr/bin/env python

# ros
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Pose, Twist, Transform
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, SetBool
from sensor_msgs.msg import Imu
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from unreal_cv_ros.msg import UeSensorRaw
from unreal_cv_ros.srv import GetCameraParams
from sensor_msgs.msg import PointCloud2

import tf

# Python
import sys
import math
import numpy as np
import time
from collections import deque

# Plotting
from matplotlib import pyplot as plt


class SimulationManager:

    def __init__(self):
        '''  Initialize ros node and read params '''
        # Parse parameters
        self.ns_gazebo = rospy.get_param('~ns_gazebo', "/gazebo")   
        self.ns_mav = rospy.get_param('~ns_mav', "/firefly")
        self.ns_planner = rospy.get_param('~ns_planner', "/firefly/random_planner")

        self.regulate = rospy.get_param('~regulate', False)     # Manage odom throughput for unreal_ros_client
        self.monitor = rospy.get_param('~monitor', False)       # Measure performance of unreal pipeline
        self.horizon = rospy.get_param('~horizon', 10)          # How many messages are kept for evaluation
        self.planner_delay = rospy.get_param('~delay', 0.0)     # Waiting time until the planner is launched

        if self.monitor:
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
            self.ue_out_sub = rospy.Subscriber("ue_out_in", PointCloud2, self.mon_out_callback, queue_size=10)

            # Printing service
            rospy.Service('~display_monitor', Empty, self.mon_print_handle)

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
        rospy.wait_for_service("get_camera_params")
        rospy.loginfo("Waiting for unreal client to setup ... done.")

        # Wait for planner
        rospy.loginfo("Waiting for planner to be ready...")
        rospy.wait_for_service(self.ns_planner + "/toggle_running")

        if self.planner_delay > 0:
            rospy.loginfo("Waiting for planner to be ready... done. Launch in %d seconds.", self.planner_delay)
            rospy.sleep(self.planner_delay)
        else:
            rospy.loginfo("Waiting for planner to be ready... done.")

        # Launch planner (by service)
        run_planner_srv = rospy.ServiceProxy(self.ns_planner + "/toggle_running", SetBool)
        run_planner_srv(True)
        rospy.loginfo("Succesfully started the simulation!")

    def mon_raw_callback(self, ros_data):
        # Real-time rate
        time_real = time.time()
        time_ros = rospy.get_time()
        if self.mon_client_prev_real is None:
            self.mon_client_prev_real = time_real
            self.mon_client_prev_ros = time_ros
            return

        self.mon_client_rate_real.append(time_real-self.mon_client_prev_real)
        self.mon_client_prev_real = time_real

        # Ros-time rate
        self.mon_client_rate_ros.append(time_ros-self.mon_client_prev_ros)
        self.mon_client_prev_ros = time_ros

        # Sensor delay
        self.mon_client_delay_ros.append(time_ros - ros_data.header.stamp.to_sec())

        # Visualization time
        self.mon_client_time.append(time_ros)

    def mon_out_callback(self, ros_data):
        # Real-time rate
        time_real = time.time()
        time_ros = rospy.get_time()
        if self.mon_sensor_prev_real is None:
            self.mon_sensor_prev_real = time_real
            self.mon_sensor_prev_ros = time_ros
            return

        self.mon_sensor_rate_real.append(time_real - self.mon_sensor_prev_real)
        self.mon_sensor_prev_real = time_real

        # Ros-time rate
        self.mon_sensor_rate_ros.append(time_ros - self.mon_sensor_prev_ros)
        self.mon_sensor_prev_ros = time_ros

        # Sensor delay
        self.mon_sensor_delay_ros.append(time_ros - ros_data.header.stamp.to_sec())

        # Visualization time
        self.mon_sensor_time.append(time_ros)

    def mon_print_handle(self, _):
        print("="*10 + " performance monitor" + "="*10)
        print(" Name: mean - std - min - max [s]")
        values = np.array(self.mon_client_rate_real)
        if len(values) > 0:
            print("Client rate (real): {0:.2f} - {1:.2f} - {2:.2f} - {3:.2f}"
                  .format(np.mean(values), np.std(values), np.min(values), np.max(values)))

        values = np.array(self.mon_client_rate_ros)
        if len(values) > 0:
            print("Client rate (ros):  {0:.2f} - {1:.2f} - {2:.2f} - {3:.2f}"
                  .format(np.mean(values), np.std(values), np.min(values), np.max(values)))
        values = np.array(self.mon_client_delay_ros)
        if len(values) > 0:
            print("Client delay (ros): {0:.2f} - {1:.2f} - {2:.2f} - {3:.2f}"
                  .format(np.mean(values), np.std(values), np.min(values), np.max(values)))
        values = np.array(self.mon_sensor_rate_real)
        if len(values) > 0:
            print("Sensor rate (real): {0:.2f} - {1:.2f} - {2:.2f} - {3:.2f}"
                  .format(np.mean(values), np.std(values), np.min(values), np.max(values)))
        values = np.array(self.mon_sensor_rate_ros)
        if len(values) > 0:
            print("Sensor rate (ros):  {0:.2f} - {1:.2f} - {2:.2f} - {3:.2f}"
                  .format(np.mean(values), np.std(values), np.min(values), np.max(values)))
        values = np.array(self.mon_sensor_delay_ros)
        if len(values) > 0:
            print("Sensor delay (ros): {0:.2f} - {1:.2f} - {2:.2f} - {3:.2f}"
                  .format(np.mean(values), np.std(values), np.min(values), np.max(values)))
        print("=" * 40)


if __name__ == '__main__':
    rospy.init_node('simulation_manager', anonymous=True)
    sm = SimulationManager()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down simulation_manager")
