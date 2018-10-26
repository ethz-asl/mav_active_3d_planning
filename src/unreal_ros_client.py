#!/usr/bin/env python

# Unrealcv
from unrealcv import client

# ros
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from unreal_cv_ros.msg import UeSensorRaw
import tf

# Image conversion
import StringIO
import PIL.Image
from cv_bridge import CvBridge

# Python
import sys
import math
import numpy as np


class unreal_ros_client:

    def __init__(self):
        '''  Initialize ros node, unrealcv client and params '''
        self.ready = False
        self.pub = rospy.Publisher("ue_sensor_raw", UeSensorRaw, queue_size=10)
        self.bridge = CvBridge()
        rospy.on_shutdown(self.reset_client)

        # Setup client
        client.connect()
        if client.isconnected():
            status = client.request('vget /unrealcv/status')
            if status is not None:
                rospy.loginfo("Unrealcv client status:\n" + status)
            else:
                rospy.logfatal("Error addressing the unrealcv client. Please restart the game.")
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
        self.coord_origin = np.array([float(location[0]), float(location[1]), float(location[2])])

        # Read in params
        self.test = rospy.get_param('~test', False)      # True: navigate in game, false: navigate by odom messages.

        # Finish setup
        if self.test:
            self.previous_pose = None
            self.tf_br = tf.TransformBroadcaster()
            rospy.Timer(rospy.Duration(0.1), self.test_callback)  # 10 Hz
        else:
            self.sub = rospy.Subscriber("odometry", Odometry, self.odom_callback, queue_size=1)
            self.ready = True
        rospy.loginfo("unreal_ros_client is ready...")

    def odom_callback(self, ros_data):
        ''' Produce images for given odometry '''
        if rospy.is_shutdown() or not self.ready:
            return

        # Get pose
        pose = ros_data.pose.pose
        position, orientation = self.transform_to_unreal(pose)

        # Set camera in unrealcv
        position = position + self.coord_origin
        client.request('vset /camera/0/location {0} {1} {2}'.format(position[0], position[1], position[2]))
        client.request('vset /camera/0/rotation {0} {1} {2}'.format(orientation[0], orientation[1], orientation[2]))
        self.publish_images()

    def test_callback(self, ros_data):
        ''' Produce images and broadcast odometry from unreal in-game controlled camera movement '''
        if rospy.is_shutdown():
            return

        # Get current UE pose
        location = client.request('vget /camera/0/location')
        location = str(location).split(' ')
        location = np.array([float(location[0]), float(location[1]), float(location[2])])
        orientation = client.request('vget /camera/0/rotation')
        orientation = str(orientation).split(' ')
        orientation = np.array([float(orientation[0]), float(orientation[1]), float(orientation[2])])
        pose = np.concatenate((location, orientation), axis=None)

        # Update only if there is a new view
        if np.array_equal(pose, self.previous_pose):
            return
        else:
            self.previous_pose = pose

        # Publish images and transformation
        self.publish_images()
        location, orientation = self.transform_from_unreal(pose)
        self.tf_br.sendTransform(location, orientation, rospy.Time.now(), "camera_link", "world")

    def publish_images(self):
        ''' Produce and publish images '''
        # Retrieve color image
        res = client.request('vget /camera/0/lit png')
        img_color = PIL.Image.open(StringIO.StringIO(res))
        img_color = np.asarray(img_color)

        # Retrieve depth image
        res = client.request('vget /camera/0/depth npy')
        img_depth = np.load(StringIO.StringIO(res))

        # Publish image data
        msg = UeSensorRaw()
        msg.header.stamp = rospy.Time.now()
        msg.color_image = self.bridge.cv2_to_imgmsg(img_color, "8UC4")      # 4 channel (RGBA) color uint8 encoding
        msg.depth_image = self.bridge.cv2_to_imgmsg(img_depth, "32FC1")     # float encoding
        msg.camera_info = self.camera_info
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
        orientation = [-orientation[0], -orientation[1], -orientation[2], orientation[3]]

        # Transform to unreal coordinate units
        position = position * 100  # default is cm
        orientation = orientation * 180 / math.pi  # default is deg

        # Transform to pitch, yaw, roll
        (r, p, y) = tf.transformations.euler_from_quaternion(orientation)
        orientation = np.array([p, -y, r])
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
        position = pose[[0, 1, 2]]
        orientation = pose[[3, 4, 5]]

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
