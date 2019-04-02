#!/usr/bin/env python

# ros
import rospy
from nav_msgs.msg import Odometry
from collections import deque
import tf


# Python
import math
import numpy as np


class GPSSimulator:

    def __init__(self):
        '''  Initialize ros node and read params '''
        # Parse parameters
        self.apply_noise = rospy.get_param('~apply_noise', True)
        self.position_uncertainty = rospy.get_param('~position_uncertainty', 0.03)      # [m]
        self.roll_pitch_uncertainty = rospy.get_param('~roll_pitch_uncertainty', 1)     # [deg]
        self.yaw_uncertainty = rospy.get_param('~yaw_uncertainty', 3)                   # [deg]
        self.gaussian_noise = rospy.get_param('~gaussian_noise', False)  # True: Gaussian with stddev, False: Linear
        self.crop_frequency = rospy.get_param('~crop_frequency', 25.0)        # Hz

        # constants
        self.uncertainty = np.array([self.position_uncertainty] * 3 + [self.roll_pitch_uncertainty] * 2 +
                                    [self.yaw_uncertainty])
        self.measure_length = 20

        # variables
        self.times = deque([0.0]*self.measure_length, self.measure_length)
        self.previous_time = rospy.Time.now()

        # publisher
        self.pub = rospy.Publisher("odometry_out", Odometry, queue_size=10)
        self.sub = rospy.Subscriber("odometry_in", Odometry, self.odom_callback, queue_size=1, buff_size=(2 ** 24))

    def odom_callback(self, ros_data):
        ''' Apply some artificial noise to the odom msg and republish. '''
        now = rospy.Time.now()
        time_diff = (now - self.previous_time).to_sec()
        if float(self.measure_length) / (np.sum(np.array(self.times)) + time_diff) > self.crop_frequency:
            return
        self.times.append(time_diff)
        self.previous_time = now

        if self.apply_noise:
            if self.gaussian_noise:
                offset = np.random.normal(np.zeros(np.shape(self.uncertainty)), self.uncertainty)
            else:
                offset = np.random.sample(np.shape(self.uncertainty)) * self.uncertainty

            orientation = [ros_data.pose.pose.orientation.x, ros_data.pose.pose.orientation.y,
                           ros_data.pose.pose.orientation.z, ros_data.pose.pose.orientation.w]
            (r, p, y) = tf.transformations.euler_from_quaternion(orientation)

            r = self.add_angle(r, offset[3] / 180.0 * math.pi)
            p = self.add_angle(p, offset[4] / 180.0 * math.pi)
            y = self.add_angle(y, offset[5] / 180.0 * math.pi)

            orientation = tf.transformations.quaternion_from_euler(r, p, y)

            ros_data.pose.pose.position.x = ros_data.pose.pose.position.x + offset[0]
            ros_data.pose.pose.position.y = ros_data.pose.pose.position.y + offset[1]
            ros_data.pose.pose.position.z = ros_data.pose.pose.position.z + offset[2]
            ros_data.pose.pose.orientation.x = orientation[0]
            ros_data.pose.pose.orientation.y = orientation[1]
            ros_data.pose.pose.orientation.z = orientation[2]
            ros_data.pose.pose.orientation.w = orientation[3]

        self.pub.publish(ros_data)

    @staticmethod
    def add_angle(angle, summand):
        angle += summand
        if angle > 2.0 * math.pi:
            angle -= 2.0 * math.pi
        elif angle < 0:
            angle += 2.0 * math.pi
        return angle


if __name__ == '__main__':
    rospy.init_node('gps_odometry_simulator', anonymous=True)
    gps = GPSSimulator()
    rospy.spin()

