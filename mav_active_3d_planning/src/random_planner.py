#!/usr/bin/env python

# ros
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from unreal_cv_ros.msg import UeSensorRaw, UeSensorRawFast
import tf


# Python
import sys
import math
import numpy as np
import time


class unreal_ros_client:

    def __init__(self):
        '''  Initialize ros node, unrealcv client and params '''
        self.ready = False
        rospy.on_shutdown(self.reset_client)

        # Read in params
        self.mode = rospy.get_param('~mode', "standard")  # Client mode (test, standard, fast, fast2)
        self.collision_on = rospy.get_param('~collision_on', True)  # Check for collision
        self.collision_tolerance = rospy.get_param('~collision_tol', 10)  # Distance threshold in UE units

        # Select client mode
        mode_types = {'standard': 'standard', 'fast': 'fast', 'test': 'test', 'fast2': 'fast2'}
        selected = mode_types.get(self.mode, 'NotFound')
        if selected == 'NotFound':
            warning = "Unknown client mode '" + self.mode + "'. Implemented modes are: " + \
                      "".join(["'" + m + "', " for m in mode_types])
            rospy.logfatal(warning[:-2])

        # Setup unrealcv client
        client.connect()
        if client.isconnected():
            status = client.request('vget /unrealcv/status')
            if status is not None:
                rospy.loginfo("Unrealcv client status:\n" + status)
            else:
                rospy.logfatal("Error addressing the unrealcv client. Try restarting the game.")
                return
        else:
            rospy.logfatal("No unreal game running to connect to. Please start a game before launching the node.")
            return

        # Setup camera parameters from unrealcv config
        loc1 = status.find('Width:')
        loc2 = status.find('Height:')
        loc3 = status.find('FOV:')
        loc4 = status.find('EnableInput:')

        width = int(status[loc1 + 7:loc2])
        height = int(status[loc2 + 8:loc3])
        fov = float(status[loc3 + 5:loc4])

        f = width / 2 / np.tan(fov * math.pi / 180 / 2)

        self.camera_info = CameraInfo()
        self.camera_info.width = width
        self.camera_info.height = height
        self.camera_info.K = [f, 0, width/2, 0, f, height/2, 0, 0, 1]

        # Initialize relative coordinate system (so camera starts at [0, 0, 0]).
        location = client.request('vget /camera/0/location')
        location = str(location).split(' ')
        self.coord_origin = np.array([float(x) for x in location])

        # Setup mode
        if self.mode == 'test':
            self.previous_pose = None       # Update only when pose has changed
            self.bridge = CvBridge()
            self.pub = rospy.Publisher("ue_sensor_raw", UeSensorRaw, queue_size=10)
            rospy.Timer(rospy.Duration(0.01), self.test_callback)  # 100 Hz try capture frequency

        elif self.mode == 'standard':
            self.sub = rospy.Subscriber("odometry", Odometry, self.odom_callback, queue_size=1)
            self.bridge = CvBridge()
            self.pub = rospy.Publisher("ue_sensor_raw", UeSensorRaw, queue_size=10)

        elif self.mode == 'fast':
            self.sub = rospy.Subscriber("odometry", Odometry, self.fast_callback, queue_size=1)
            self.pub = rospy.Publisher("ue_sensor_raw", UeSensorRawFast, queue_size=10)
            self.previous_odom_msg = None       # Previously processed Odom message
            self.previous_loc_req = np.array([0, 0, 0])     # Requested unreal coords for collision check

        elif self.mode == 'fast2':
            # Use ros in unreal plugin?
            self.sub = rospy.Subscriber("odometry", Odometry, self.fast_callback, queue_size=1)

        # Performance measures
        self.perf_reqs_skipped = []
        self.perf_reqs_counter = 0
        self.perf_framerate_true = []
        self.perf_framerate_ros = []

        # Finish setup
        self.ready = True
        rospy.loginfo("unreal_ros_client is ready in mode: " + self.mode)


if __name__ == '__main__':
    rospy.init_node('unreal_ros_client', anonymous=False)
    uc = unreal_ros_client()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down random_planner")
