#!/usr/bin/env python

# ros
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from collections import deque
import tf

# Python
import math
import numpy as np


class GPSSimulator:

    def __init__(self):
        '''  Initialize ros node and read params '''
        # Parse parameters
        self.position_uncertainty = rospy.get_param('~position_uncertainty', 0.03)  # [m]
        self.roll_pitch_uncertainty = rospy.get_param('~roll_pitch_uncertainty', 1)  # [deg]
        self.yaw_uncertainty = rospy.get_param('~yaw_uncertainty', 3)  # [deg]
        self.crop_frequency = rospy.get_param('~crop_frequency', 25.0)  # Hz
        self.noise_model = rospy.get_param('~noise_model', "none")  # none, linear, gaussian, random_walk
        self.publish_difference = rospy.get_param('~publish_difference', True)

        # constants
        self.uncertainty = np.array([self.position_uncertainty] * 3 + [self.roll_pitch_uncertainty] * 2 +
                                    [self.yaw_uncertainty])
        self.measure_length = 20

        # variables
        self.times = deque([0.0] * self.measure_length, self.measure_length)
        self.previous_time = rospy.Time.now()

        if self.noise_model == "none":
            self.apply_noise = self.apply_noise_none
        elif self.noise_model == "linear":
            self.apply_noise = self.apply_noise_linear
        elif self.noise_model == "gaussian":
            self.apply_noise = self.apply_noise_gaussian
        elif self.noise_model == "random_walk":
            self.apply_noise = self.apply_noise_random_walk
        else:
            rospy.logerr("Unknown GPS noise model '" + self.noise_model + "'!")
            self.apply_noise = self.apply_noise_none

        # publisher
        self.pub = rospy.Publisher("odometry_out", Odometry, queue_size=10)
        self.sub = rospy.Subscriber("odometry_in", Odometry, self.odom_callback, queue_size=1, buff_size=(2 ** 24))

        if self.publish_difference:
            self.dist_diff_pub = rospy.Publisher("~distance_difference", Float32, queue_size=10)
            self.angle_diff_pub = rospy.Publisher("~angle_difference", Float32, queue_size=10)

    def odom_callback(self, ros_data):
        ''' Apply some artificial noise to the odom msg and republish. '''

        # Check wether the current frequency is above the desired one
        now = rospy.Time.now()
        time_diff = (now - self.previous_time).to_sec()
        if float(self.measure_length) / (np.sum(np.array(self.times)) + time_diff) > self.crop_frequency:
            return
        self.times.append(time_diff)
        self.previous_time = now

        new_msg = self.apply_noise(ros_data)

        if self.publish_difference:
            prev_ori = np.array([ros_data.pose.pose.orientation.x, ros_data.pose.pose.orientation.y,
                                 ros_data.pose.pose.orientation.z, ros_data.pose.pose.orientation.w])
            new_ori = np.array([new_msg.pose.pose.orientation.x, new_msg.pose.pose.orientation.y,
                                new_msg.pose.pose.orientation.z, new_msg.pose.pose.orientation.w])
            prev_pos = np.array([ros_data.pose.pose.position.x, ros_data.pose.pose.position.y,
                                 ros_data.pose.pose.position.z])
            new_pos = np.array([new_msg.pose.pose.position.x, new_msg.pose.pose.position.y,
                                new_msg.pose.pose.position.z])
            self.dist_diff_pub.publish((np.linalg.norm(prev_pos - new_pos)))
            self.angle_diff_pub.publish(2.0*math.acos(min(math.fabs(np.dot(prev_ori, new_ori)), 1.0)))

        self.pub.publish(new_msg)

    @staticmethod
    def apply_noise_none(ros_data):
        # Continue with ground truth
        return ros_data

    def apply_noise_linear(self, ros_data):
        # apply uniform random offsets (jumpy!)
        offset = (np.random.sample(np.shape(self.uncertainty)) - 0.5) * 2.0 * self.uncertainty

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
        return ros_data

    def apply_noise_gaussian(self, ros_data):
        # apply gaussian random offsets, bounded by 2x stddev (jumpy!)
        offset = np.random.normal(np.zeros(np.shape(self.uncertainty)), self.uncertainty)
        offset = np.minimum(offset, self.uncertainty * 2.0)
        offset = np.maximum(offset, self.uncertainty * -2.0)

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
        return ros_data

    def apply_noise_random_walk(self, ros_data):
        # Random walk in 3d, bounded by position_uncertainty radius sphere.
        # The
        # double theta = 2 * M_PI * uniform01(generator);       # random walk direction in 3d
        #         double phi = acos(1 - 2 * uniform01(generator));
        #         double x = sin(phi) * cos(theta);
        #         double y = sin(phi) * sin(theta);
        #         double z = cos(phi);
        offset = np.random.normal(np.zeros(np.shape(self.uncertainty)), self.uncertainty)

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
        return ros_data

    @staticmethod
    def add_angle(angle, to_add):
        angle += to_add
        if angle > 2.0 * math.pi:
            angle -= 2.0 * math.pi
        elif angle < 0:
            angle += 2.0 * math.pi
        return angle


if __name__ == '__main__':
    rospy.init_node('gps_odometry_simulator', anonymous=True)
    gps = GPSSimulator()
    rospy.spin()
