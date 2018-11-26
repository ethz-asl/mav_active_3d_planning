#!/usr/bin/env python

# ros
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Pose, Twist, Transform
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu
from gazebo_msgs.srv import SetModelState
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
        self.mav_name = rospy.get_param('~mav_name', "NoNameFound")
        if self.mav_name == "NoNameFound":
            rospy.logfatal("Parameter mav_name is required!")

        self.regulate = rospy.get_param('~regulate', False)     # Manage odom throughput for unreal_ros_client
        self.monitor = rospy.get_param('~monitor', False)       # Measure performance of unreal pipeline
        self.framerate = rospy.get_param('~framerate', 5)       # Set approximate framerate for regulation
        self.horizon = rospy.get_param('~horizon', 10)         # How many messages are kept for evaluation

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
            rospy.Timer(rospy.Duration(1.0), self.mon_print_handle)

        if self.regulate:
            # Subscribers and publishers
            self.odom_sub = rospy.Subscriber("odom_in", Odometry, self.odom_callback, queue_size=10)
            self.odom_pub = rospy.Publisher("odom_out", Odometry, queue_size=10)

        # Run the startup
        self.launch_simulation()

    def launch_simulation(self):
        # Wait for Gazebo services
        rospy.loginfo("Starting simulation setup coordination...")
        rospy.wait_for_service("/gazebo/unpause_physics")
        rospy.wait_for_service("/gazebo/set_model_state")

        # Prepare initialization trajectory command
        traj_pub = rospy.Publisher("/" + self.mav_name + "/command/trajectory", MultiDOFJointTrajectory, queue_size=10)
        traj_msg = MultiDOFJointTrajectory()
        traj_msg.joint_names = ["base_link"]
        transforms = Transform()
        transforms.translation.x = 0.0
        transforms.translation.y = 0.0
        transforms.translation.z = 0.0
        point = MultiDOFJointTrajectoryPoint([transforms], [Twist()], [Twist()], rospy.Duration(0))
        traj_msg.points.append(point)

        # Prepare initialization setModelState service
        set_model_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        model_state = ModelState(self.mav_name, Pose(), Twist(), "world")

        # Wake up gazebo
        rospy.loginfo("Waiting for gazebo to wake up ...")
        unpause_srv = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        while not unpause_srv():
            rospy.sleep(0.1)
        rospy.loginfo("Waiting for gazebo to wake up ... done!")

        # Wait for drone to spawn (imu is publishing)
        rospy.loginfo("Waiting for MAV to spawn ...")
        rospy.wait_for_message("/" + self.mav_name +"/imu", Imu)

        # Initialize drone stable at [0, 0, 0]
        traj_msg.header.stamp = rospy.Time.now()
        traj_pub.publish(traj_msg)
        set_model_srv(model_state)
        rospy.loginfo("Waiting for MAV to spawn ... done!")

        # Wait for unreal client
        rospy.loginfo("Waiting for unreal client to setup ...")
        rospy.wait_for_service("get_camera_params")
        rospy.loginfo("Waiting for unreal client to setup ... done!")

        # Finish
        rospy.loginfo("Succesfully started the simulation!")

    def odom_callback(self, ros_data):
        return

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

    def mon_display(self, _):
        print(self.mon_client_time)
        print(self.mon_client_rate_real)

        client_time = np.array(self.mon_client_time)
        sensor_time = np.array(self.mon_sensor_time)

        print(np.shape(client_time))
        print(np.shape(np.array(self.mon_client_rate_real)))

        print(len(client_time), len(self.mon_client_rate_real), len(self.mon_client_delay_ros), len(self.mon_client_rate_ros))
        print(len(sensor_time), len(self.mon_sensor_rate_real), len(self.mon_sensor_delay_ros), len(self.mon_sensor_rate_ros))


        # plt.subplot(3, 1, 1)
        # plt.plot(client_time, np.array(self.mon_client_rate_real), 'r-', lw=2)
        # plt.plot(sensor_time, np.array(self.mon_sensor_rate_real), 'b-', lw=2)
        # plt.xlabel('Simulated Time [s]')
        # plt.ylabel('Real Cycle Time [S]')
        # plt.title('Real cycle time of the client (red) and full sensor (blue)')
        # plt.grid(True)
        #
        # plt.subplot(3, 1, 2)
        # plt.plot(client_time, np.array(self.mon_client_rate_ros), 'r-', lw=2)
        # plt.plot(sensor_time, np.array(self.mon_sensor_rate_ros), 'b-', lw=2)
        # plt.xlabel('Simulated Time [s]')
        # plt.ylabel('Simulated Cycle Time [S]')
        # plt.title('Simulated cycle time of the client (red) and full sensor (blue)')
        # plt.grid(True)
        #
        # plt.subplot(3, 1, 3)
        # plt.plot(client_time, np.array(self.mon_client_delay_ros), 'r-', lw=2)
        # plt.plot(sensor_time, np.array(self.mon_sensor_delay_ros), 'b-', lw=2)
        # plt.xlabel('Simulated Time [s]')
        # plt.ylabel('Simulated Delay [S]')
        # plt.title('Simulated delay of the client (red) and full sensor (blue)')
        # plt.grid(True)
        #
        # if len(sensor_time) > 0:
        #     plt.subplot(3, 2, 2)
        #     plt.plot(sensor_time, np.array(self.mon_sensor_rate_real), 'b-', lw=2)
        #     plt.xlabel('Simulated Time [s]')
        #     plt.ylabel('Real Cycle Time [S]')
        #     plt.title('Real cycle time of the client (red) and full sensor (blue)')
        #     plt.grid(True)
        #     plt.subplot(3, 2, 4)
        #     plt.plot(sensor_time, np.array(self.mon_sensor_rate_ros), 'b-', lw=2)
        #     plt.xlabel('Simulated Time [s]')
        #     plt.ylabel('Simulated Cycle Time [S]')
        #     plt.title('Simulated cycle time of the client (red) and full sensor (blue)')
        #     plt.grid(True)
        #     plt.subplot(3, 2, 6)
        #     plt.plot(sensor_time, np.array(self.mon_sensor_delay_ros), 'b-', lw=2)
        #     plt.xlabel('Simulated Time [s]')
        #     plt.ylabel('Simulated Delay [S]')
        #     plt.title('Simulated delay of the client (red) and full sensor (blue)')
        #     plt.grid(True)
        #
        # if len(client_time) > 0:
        #     plt.subplot(3, 2, 1)
        #     plt.plot(client_time, np.array(self.mon_client_rate_real), 'r-', lw=2)
        #     plt.subplot(3, 2, 3)
        #     plt.plot(client_time, np.array(self.mon_client_rate_ros), 'r-', lw=2)
        #     plt.subplot(3, 2, 5)
        #     plt.plot(client_time, np.array(self.mon_client_delay_ros), 'r-', lw=2)
        #     plt.tight_layout()
        #     # plt.draw()
        #     plt.show(block=False)


if __name__ == '__main__':
    rospy.init_node('simulation_manager', anonymous=True)
    sm = SimulationManager()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down simulation_manager")
