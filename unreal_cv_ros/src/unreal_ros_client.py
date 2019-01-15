#!/usr/bin/env python

# Unrealcv
from unrealcv import client

# ros
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from unreal_cv_ros.msg import UeSensorRaw
from unreal_cv_ros.srv import GetCameraParams, GetCameraParamsResponse
import tf

# Python
import sys
import math
import numpy as np


class UnrealRosClient:

    def __init__(self):
        '''  Initialize ros node, unrealcv client and params '''
        rospy.on_shutdown(self.reset_client)

        # Read in params
        self.mode = rospy.get_param('~mode', "standard")  # Client mode (test, standard, fast, fast2)
        self.collision_on = rospy.get_param('~collision_on', True)  # Check for collision
        self.collision_tolerance = rospy.get_param('~collision_tol', 10)  # Distance threshold in UE units
        self.publish_tf = rospy.get_param('~publish_tf', False)  # If true publish the camera transformation in tf

        # Select client mode
        mode_types = {'standard': 'standard', 'fast': 'fast', 'test': 'test'}
        selected = mode_types.get(self.mode, 'NotFound')
        if selected == 'NotFound':
            warning = "Unknown client mode '" + self.mode + "'. Implemented modes are: " + \
                      "".join(["'" + m + "', " for m in mode_types])
            rospy.logfatal(warning[:-2])

        # Setup unrealcv client
        client.connect()
        if not client.isconnected():
            rospy.logfatal("No unreal game running to connect to. Please start a game before launching the node.")

        status = client.request('vget /unrealcv/status')
        if status is None:
            rospy.logfatal("Error addressing the unrealcv client. Try restarting the game.")

        rospy.loginfo("Unrealcv client status:\n" + status)

        # Setup camera parameters from unrealcv config
        loc_width = status.find('Width:')
        loc_height = status.find('Height:')
        loc_fov = status.find('FOV:')
        loc_end = status.find('EnableInput:')
        width = int(status[loc_width + 7:loc_height])
        height = int(status[loc_height + 8:loc_fov])
        fov = float(status[loc_fov + 5:loc_end])
        f = width / 2 / np.tan(fov * math.pi / 180 / 2)
        self.camera_params = [width, height, f]

        # Initialize relative coordinate system (so camera starts at [0, 0, 0] position and [0, 0, yaw]).
        location = client.request('vget /camera/0/location')
        location = str(location).split(' ')
        self.coord_origin = np.array([float(x) for x in location])
        rot = client.request('vget /camera/0/rotation')
        self.coord_yaw = np.array([float(x) for x in str(rot).split(' ')])[1]
        client.request("vset /camera/0/rotation 0 {0:f} 0".format(self.coord_yaw))

        # tf broadcaster
        if self.mode == 'test' or self.publish_tf:
            self.tf_br = tf.TransformBroadcaster()                  # Publish camera transforms from game

        # Setup mode
        if self.mode == 'test':
            self.previous_pose = None                               # Update only when pose has changed
            rospy.Timer(rospy.Duration(0.01), self.test_callback)   # 100 Hz try capture frequency

        elif self.mode == 'standard':
            self.sub = rospy.Subscriber("odometry", Odometry, self.odom_callback, queue_size=1, buff_size=2 ** 24)

        elif self.mode == 'fast':
            self.sub = rospy.Subscriber("odometry", Odometry, self.fast_callback, queue_size=1, buff_size=2 ** 24)
            self.previous_odom_msg = None                   # Previously processed Odom message
            self.previous_loc_req = np.array([0, 0, 0])     # Requested unreal coords for collision check
            if not self.collision_on:
                self.collision_tolerance = -1

        # Finish setup
        self.pub = rospy.Publisher("ue_sensor_raw", UeSensorRaw, queue_size=10)
        self.camera_params_srv = rospy.Service('get_camera_params', GetCameraParams, self.get_camera_params_handle)
        rospy.loginfo("unreal_ros_client is ready in %s mode." % self.mode)

    def fast_callback(self, ros_data):
        ''' Use the custom unrealcv command to get images and collision checks. vget UECVROS command returns the images
        and then sets the new pose, therefore the delay. '''
        # Get pose in unreal coords
        rospy.sleep(0.3)
        position, orientation = self.transform_to_unreal(ros_data.pose.pose)

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
                idx = int(len(result) / 2)  # Half of the bytes are the 4channel color img, half the depth
                msg = UeSensorRaw()
                msg.header.stamp = self.previous_odom_msg.header.stamp
                msg.color_data = result[:idx]
                msg.depth_data = result[idx:]
                self.pub.publish(msg)
            else:
                rospy.logerr("Unknown return format from unrealcv {0}".format(type(result)))

        self.previous_odom_msg = ros_data
        self.previous_loc_req = position

        if self.publish_tf:
            self.publish_tf_data(ros_data)

    def odom_callback(self, ros_data):
        ''' Produce images for given odometry '''
        # Get pose
        position, orientation = self.transform_to_unreal(ros_data.pose.pose)

        # Set camera in unrealcv
        if self.collision_on:
            client.request('vset /camera/0/moveto {0:f} {1:f} {2:f}'.format(*position))

            # Check collision
            position_eff = client.request('vget /camera/0/location')
            position_eff = np.array([float(x) for x in str(position_eff).split(' ')])
            if np.linalg.norm(position - position_eff) >= self.collision_tolerance:
                rospy.logwarn("MAV collision detected!")
        else:
            client.request("vset /camera/0/location {0:f} {1:f} {2:f}".format(*position))

        client.request("vset /camera/0/rotation {0:f} {1:f} {2:f}".format(*orientation))

        # Generate images
        self.publish_images(ros_data.header.stamp)

        if self.publish_tf:
            self.publish_tf_data(ros_data)

    def test_callback(self, _):
        ''' Produce images and broadcast odometry from unreal in-game controlled camera movement '''
        # Get current UE pose
        position = client.request('vget /camera/0/location')
        position = str(position).split(' ')
        position = np.array([float(x) for x in position])
        orientation = client.request('vget /camera/0/rotation')
        orientation = str(orientation).split(' ')
        orientation = np.array([float(x) for x in orientation])
        pose = [position, orientation]

        # Update only if there is a new view, publish images
        if np.array_equal(pose, self.previous_pose):
            return

        self.previous_pose = pose
        timestamp = rospy.Time.now()
        self.publish_images(timestamp)

        position, orientation = self.transform_from_unreal(pose)
        self.tf_br.sendTransform((position[0], position[1], position[2]),
                                 (orientation[0], orientation[1], orientation[2], orientation[3]),
                                 timestamp, "camera_link", "world")

    def publish_images(self, header_stamp):
        ''' Produce and publish images for test and standard mode'''
        # Retrieve images
        res_color = client.request('vget /camera/0/lit npy')
        res_depth = client.request('vget /camera/0/depth npy')

        # Publish data
        msg = UeSensorRaw()
        msg.header.stamp = header_stamp
        msg.color_data = res_color
        msg.depth_data = res_depth
        self.pub.publish(msg)

    def publish_tf_data(self, odom_msg):
        pos = odom_msg.pose.pose.position
        rot = odom_msg.pose.pose.orientation
        self.tf_br.sendTransform((pos.x, pos.y, pos.z), (rot.x, rot.y, rot.z, rot.w), odom_msg.header.stamp,
                                 "camera_link", "world")

    def get_camera_params_handle(self, _):
        return GetCameraParamsResponse(self.camera_params[0], self.camera_params[1], self.camera_params[2])

    def transform_to_unreal(self, pose):
        '''
        Transform from ros to default unreal coordinates.
        Input:      ros pose as [x, y, z] array + quaternion array (as from pose msg)
        Output:     position ([x, y, z] array, in unreal coordinates)
                    orientation ([pitch, yaw, roll] array in unreal coordinates)
        '''
        # Read out vectors
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        # Transform to relative Unreal coordinate sys rotation
        yaw = self.coord_yaw / 180 * math.pi
        q_rot = tf.transformations.quaternion_from_euler(0, 0, yaw)
        orientation = tf.transformations.quaternion_multiply(q_rot, orientation)
        position = np.array([math.cos(yaw)*x - math.sin(yaw)*y, math.sin(yaw)*x + math.cos(yaw)*y, z])

        # Invert y axis
        position[1] = -position[1]

        # Invert rotation for left handed coordinates
        orientation = np.array([-orientation[0], -orientation[1], -orientation[2], orientation[3]])

        # Transform to pitch, yaw, roll
        (r, p, y) = tf.transformations.euler_from_quaternion(orientation)
        orientation = np.array([p, y, r])

        # Transform to unreal coordinate units
        position = position * 100  # default is cm
        orientation = orientation * 180 / math.pi  # default is deg

        # Transform to relative unreal coordinate sys position
        position = position + self.coord_origin
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
        (x, y, z, w) = tf.transformations.quaternion_from_euler(orientation[2], orientation[0], orientation[1])
        orientation = np.array([x, y, z, w])

        # Invert y axis
        position[1] = -position[1]

        # Invert rotation for left handed coordinates
        orientation = [-orientation[0], -orientation[1], -orientation[2], orientation[3]]
        return position, orientation

    @staticmethod
    def reset_client():
        rospy.loginfo('On unreal_ros_client shutdown: disconnecting unrealcv client.')
        client.disconnect()


if __name__ == '__main__':
    rospy.init_node('unreal_ros_client', anonymous=False)  # Currently only 1 Client at a time is supported by unrealcv
    uc = UnrealRosClient()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down unreal_ros_client")
