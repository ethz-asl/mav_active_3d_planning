#!/usr/bin/env python

# Unrealcv
from unrealcv import client

# ros
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from unreal_cv_ros.msg import UeSensorRaw
import tf

# Python
import sys
import math
import numpy as np
import time


class UnrealRosClient:

    def __init__(self):
        '''  Initialize ros node, unrealcv client and params '''
        rospy.on_shutdown(self.reset_client)

        # Read in params
        self.mode = rospy.get_param('~mode', "standard")  # Client mode (test, standard, fast, fast2)
        self.collision_on = rospy.get_param('~collision_on', True)  # Check for collision
        self.publish_tf = rospy.get_param('~publish_tf', False)  # If true publish the camera transformation in tf
        self.slowdown = rospy.get_param('~slowdown', 0.0)  # Artificially slow down rate for UE to finish rendering
        self.camera_id = rospy.get_param('~camera_id', 0)  # CameraID for unrealcv compatibility (usually use 0)
        self.queue_size = rospy.get_param('~queue_size', 1)   # How many requests are kept

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
        rospy.set_param('~camera_params', {'width': float(width), 'height': float(height), 'focal_length': float(f)})

        # Initialize relative coordinate system (so camera starts at [0, 0, 0] position and [0, 0, yaw]).
        location = client.request('vget /camera/%i/location' % self.camera_id)
        self.coord_origin = np.array([float(x) for x in str(location).split(' ')])
        rot = client.request('vget /camera/%i/rotation' % self.camera_id)
        self.coord_yaw = float(str(rot).split(' ')[1])
        client.request("vset /camera/{0:d}/rotation 0 {1:f} 0".format(self.camera_id, self.coord_yaw))

        # tf broadcaster
        if self.mode == 'test' or self.publish_tf:
            self.tf_br = tf.TransformBroadcaster()  # Publish camera transforms in tf

        # Setup mode
        if self.mode == 'test':
            self.previous_pose = None                               # Update only when pose has changed
            rospy.Timer(rospy.Duration(0.01), self.test_callback)   # 100 Hz try capture frequency

        elif self.mode == 'standard':
            self.sub = rospy.Subscriber("odometry", Odometry, self.odom_callback, queue_size=self.queue_size,
                                        buff_size=(2**24) * self.queue_size)
            # The buffersize needs to be large enough to fit all messages, otherwise strange things happen
            self.previous_odom_msg = None                   # Previously processed Odom message
            self.collision_tolerance = rospy.get_param('~collision_tol', 10)  # Distance threshold in UE units

        elif self.mode == 'fast':
            self.sub = rospy.Subscriber("odometry", Odometry, self.fast_callback, queue_size=self.queue_size,
                                        buff_size=(2**24) * self.queue_size)
            self.previous_odom_msg = None                   # Previously processed Odom message

        # Finish setup
        self.pub = rospy.Publisher("~ue_sensor_raw", UeSensorRaw, queue_size=10)
        if self.collision_on:
            self.collision_pub = rospy.Publisher("~collision", String, queue_size=10)
        rospy.loginfo("unreal_ros_client is ready in %s mode." % self.mode)

    def fast_callback(self, ros_data):
        ''' Use the custom unrealcv command to get images and collision checks. vget UECVROS command returns the images
        and then sets the new pose, therefore the delay. '''
        # Slowdown to give more rendering time to the unreal engine
        time.sleep(self.slowdown)

        # Get pose in unreal coords
        position, orientation = self.transform_to_unreal(ros_data.pose.pose)

        # Call the plugin (takes images and then sets the new position)
        args = np.concatenate((position, orientation, np.array([float(self.collision_on)])), axis=0)
        result = client.request("vget /uecvros/full" + "".join([" {0:f}".format(x) for x in args]) + " " +
                                str(self.camera_id))

        if self.previous_odom_msg is not None:
            # This is not the initialization step
            if isinstance(result, unicode):
                # Check for collision or errors
                if result == "Collision detected!":
                    self.on_collision()
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

        if self.publish_tf:
            self.publish_tf_data(ros_data)

    def odom_callback(self, ros_data):
        ''' Produce images for given odometry '''
        # Slowdown to give more rendering time to the unreal engine (should not be necessary in normal mode)
        time.sleep(self.slowdown)

        # Get pose in unreal coords
        position, orientation = self.transform_to_unreal(ros_data.pose.pose)

        # Generate images
        if self.previous_odom_msg is not None:
            # This is not the initialization step
            self.publish_images(self.previous_odom_msg.header.stamp)

        if self.publish_tf:
            self.publish_tf_data(ros_data)

        self.previous_odom_msg = ros_data

        # Set camera in unrealcv
        if self.collision_on:
            client.request('vset /camera/{0:d}/moveto {1:f} {2:f} {3:f}'.format(self.camera_id, *position))

            # Check collision
            position_eff = client.request('vget /camera/%d/location' % self.camera_id)
            position_eff = np.array([float(x) for x in str(position_eff).split(' ')])
            if np.linalg.norm(position - position_eff) >= self.collision_tolerance:
                self.on_collision()
        else:
            client.request("vset /camera/{0:d}/location {1:f} {2:f} {3:f}".format(self.camera_id, *position))

        client.request("vset /camera/{0:d}/rotation {1:f} {2:f} {3:f}".format(self.camera_id, *orientation))

    def test_callback(self, _):
        ''' Produce images and broadcast odometry from unreal in-game controlled camera movement '''
        # Get current UE pose
        position = client.request('vget /camera/%d/location' % self.camera_id)
        position = np.array([float(x) for x in str(position).split(' ')])
        orientation = client.request('vget /camera/%d/rotation' % self.camera_id)
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
        res_color = client.request('vget /camera/%d/lit npy' % self.camera_id)
        res_depth = client.request('vget /camera/%d/depth npy' % self.camera_id)

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

    def transform_to_unreal(self, pose):
        '''
        Transform from ros to default unreal coordinates.
        Input:      ros pose in global frame
        Output:     position ([x, y, z] array, in unreal coordinates)
                    orientation ([pitch, yaw, roll] array in unreal coordinates)
        '''
        # Read out vectors
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        # Transformation of position to relative UE coordsys
        yaw = self.coord_yaw / 180 * math.pi
        position = np.array([math.cos(yaw) * x + math.sin(yaw) * y, math.sin(yaw) * x - math.cos(yaw) * y, z])
        position = position * 100 + self.coord_origin       # default units are cm

        # Transformation of orientation to relative UE coordsys
        q_rot = tf.transformations.quaternion_from_euler(0, 0, -yaw)
        orientation = tf.transformations.quaternion_multiply(q_rot, orientation)
        (r, p, y) = tf.transformations.euler_from_quaternion(orientation)
        orientation = np.array([-p, -y, r]) * 180 / math.pi     # default units are deg
        return position, orientation

    @staticmethod
    def transform_from_unreal(pose):
        '''
        Transform from unreal coordinates to odom coordinates (to camera_link)
        Input:      pose ([x, y, z, pitch, yaw, roll] array, in unrealcv coordinates)
        Output:     position ([x, y, z] array)
                    orientation ([x, y, z, w] quaternion)
        '''
        # Read out array
        position = pose[0]
        orientation = pose[1]

        # Transform from unreal coordinate units
        position = position / 100  # default is cm
        orientation = orientation / 180 * math.pi  # default is deg

        # Transform from pitch, yaw, roll (taking into account rotation directions
        (x, y, z, w) = tf.transformations.quaternion_from_euler(orientation[2], -orientation[0], -orientation[1])
        orientation = np.array([x, y, z, w])

        # Invert y axis
        position[1] = -position[1]
        return position, orientation

    @staticmethod
    def reset_client():
        # Free up unrealcv connection cleanly when shutting down
        rospy.loginfo('On unreal_ros_client shutdown: disconnecting unrealcv client.')
        client.disconnect()

    def on_collision(self):
        # Collision handling for all modes here
        rospy.logwarn("MAV collision detected!")
        self.collision_pub.publish(String("MAV collision detected!"))


if __name__ == '__main__':
    rospy.init_node('unreal_ros_client', anonymous=False)  # Currently only 1 Client at a time is supported by unrealcv
    uc = UnrealRosClient()
    rospy.spin()
