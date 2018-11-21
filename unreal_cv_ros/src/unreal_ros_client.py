#!/usr/bin/env python

# Unrealcv
from unrealcv import client

# ros
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from unreal_cv_ros.msg import UeSensorRaw, UeSensorRawFast
import tf

# Image conversion
import StringIO
import PIL.Image
from cv_bridge import CvBridge

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

    def fast_callback(self, ros_data):
        ''' Use the custom unrealcv command to get images and collision checks. vget UECVROS command returns the images
        and then sets the new pose, therefore the delay. '''
        # Performance Measures
        self.perf_reqs_counter += 1
        print("callback call")

        if rospy.is_shutdown() or not self.ready:
            return
        self.ready = False

        time_start = time.time()
        self.perf_reqs_skipped.append(self.perf_reqs_counter-1)
        self.perf_reqs_counter = 0

        # Get pose in unreal coords
        position, orientation = self.transform_to_unreal(ros_data.pose.pose)
        position = position + self.coord_origin

        # Call the plugin (takes images and then sets the new position)
        args = np.concatenate((position, orientation, np.array([self.collision_tolerance]), self.previous_loc_req),
                              axis=0)
        result = client.request("vget /uecvros/full" + "".join([" {0:f}".format(x) for x in args]))

        if self.previous_odom_msg is not None:
            # This is not the initialization step
            if isinstance(result, unicode):
                # Check for collision or errors
                if result[:9] == "Collision":
                    rospy.logwarn(result)
                else:
                    rospy.logerr("Unrealcv error: " + result)
            elif isinstance(result, str):
                # Successful: Publish data for previous pose
                msg = UeSensorRawFast()
                msg.header.stamp = self.previous_odom_msg.header.stamp
                msg.data = result
                msg.camera_info = self.camera_info
                msg.camera_link_pose = self.previous_odom_msg.pose.pose
                self.pub.publish(msg)
                print("published an image")
            else:
                rospy.logerr("Unknown return format from unrealcv {0}".format(type(result)))

        self.previous_odom_msg = ros_data
        self.previous_loc_req = position
        self.perf_framerate_true.append(float(time.time()-time_start))
        print(np.mean(self.perf_reqs_skipped), np.std(self.perf_reqs_skipped),
              np.mean(self.perf_framerate_true), np.std(self.perf_framerate_true))
        self.ready = True

    def odom_callback(self, ros_data):
        ''' Produce images for given odometry '''
        if rospy.is_shutdown() or not self.ready:
            return
        self.ready = False

        # Get pose
        pose = ros_data.pose.pose
        position, orientation = self.transform_to_unreal(pose)
        pose = [position, orientation]

        # Set camera in unrealcv
        position = position + self.coord_origin
        if self.collision_on:
            client.request('vset /camera/0/moveto {0} {1} {2}'.format(*position))

            # Check collision
            position_eff = client.request('vget /camera/0/location')
            position_eff = np.array([float(x) for x in str(position_eff).split(' ')])
            if np.linalg.norm(position - position_eff) >= self.collision_tolerance:
                rospy.logwarn("MAV collision detected!")
        else:
            client.request("vset /camera/0/location {0:f} {1:f} {2:f}".format(*position))

        client.request("vset /camera/0/rotation {0:f} {1:f} {2:f}".format(*orientation))

        # Generate images
        self.publish_images(pose, ros_data.header.stamp)
        self.ready = True

    def test_callback(self, _):
        ''' Produce images and broadcast odometry from unreal in-game controlled camera movement '''
        if rospy.is_shutdown() or not self.ready:
            return
        self.ready = False

        # Get current UE pose
        position = client.request('vget /camera/0/location')
        position = str(position).split(' ')
        position = np.array([float(x) for x in position])
        orientation = client.request('vget /camera/0/rotation')
        orientation = str(orientation).split(' ')
        orientation = np.array([float(x) for x in orientation])
        pose = [position, orientation]

        # Update only if there is a new view, publish images
        if not np.array_equal(pose, self.previous_pose):
            self.previous_pose = pose
            self.publish_images(pose, rospy.Time.now())

        self.ready = True

    def publish_images(self, pose, header_stamp):
        ''' Produce and publish images for test and standard mode'''
        # Retrieve color image
        res = client.request('vget /camera/0/lit png')
        img_color = PIL.Image.open(StringIO.StringIO(res))
        img_color = np.asarray(img_color)

        # Retrieve depth image
        res = client.request('vget /camera/0/depth npy')
        img_depth = np.load(StringIO.StringIO(res))

        # Retrieve camera pose
        position, orientation = self.transform_from_unreal(pose)

        # Publish data
        msg = UeSensorRaw()
        msg.header.stamp = header_stamp
        msg.color_image = self.bridge.cv2_to_imgmsg(img_color, "8UC4")      # 4 channel (RGBA) color uint8 encoding
        msg.depth_image = self.bridge.cv2_to_imgmsg(img_depth, "32FC1")     # float encoding
        msg.camera_info = self.camera_info
        msg.camera_link_pose.position.x = position[0]
        msg.camera_link_pose.position.y = position[1]
        msg.camera_link_pose.position.z = position[2]
        msg.camera_link_pose.orientation.x = orientation[0]
        msg.camera_link_pose.orientation.y = orientation[1]
        msg.camera_link_pose.orientation.z = orientation[2]
        msg.camera_link_pose.orientation.w = orientation[3]
        self.pub.publish(msg)

    @staticmethod
    def transform_to_unreal(pose):
        '''
        Transform from ros to default unreal coordinates.
        Input:      pose (ROS geometry_msgs/Pose)
        Output:     position ([x, y, z] array, in unreal coordinates)
                    orientation ([pitch, yaw, roll] array in unreal coordinates)
        '''
        # Read out array
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        # Invert y axis
        position[1] = -position[1]
        orientation[1] = -orientation[1]

        # Invert rotation for left handed coordinates
        orientation = np.array([-orientation[0], -orientation[1], -orientation[2], orientation[3]])

        # Transform to pitch, yaw, roll
        (r, p, y) = tf.transformations.euler_from_quaternion(orientation)
        orientation = np.array([-p, y, r])

        # Transform to unreal coordinate units
        position = position * 100  # default is cm
        orientation = orientation * 180 / math.pi  # default is deg
        return position, orientation

    @staticmethod
    def transform_from_unreal(pose):
        '''
        Transform from unreal coordinates to odom coordinates (to camera_link)
        Input:      pose ([x, y, z, pitch, yaw, rol] array, in unrealcv coordinates)
        Output:     position ([x, y, z] array)
                    orientation ([x, y, z, w] quaternion)
        '''
        # Read out array
        position = pose[0]
        orientation = pose[1]

        # Transform from unreal coordinate units
        position = position / 100  # default is cm
        orientation = orientation / 180 * math.pi  # default is deg

        # Transform from pitch, yaw, roll
        (x, y, z, w) = tf.transformations.quaternion_from_euler(orientation[2], -orientation[0], orientation[1])
        orientation = np.array([x, y, z, w])

        # Invert y axis
        position[1] = -position[1]
        orientation[1] = -orientation[1]

        # Invert rotation for left handed coordinates
        orientation = [-orientation[0], -orientation[1], -orientation[2], orientation[3]]
        return position, orientation

    @staticmethod
    def reset_client():
        rospy.loginfo('On unreal_ros_client shutdown: disconnecting unrealcv client.')
        client.disconnect()


if __name__ == '__main__':
    rospy.init_node('unreal_ros_client', anonymous=False)  # Currently only 1 Client at a time is supported by unrealcv
    uc = unreal_ros_client()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down unreal_ros_client")
