#!/usr/bin/env python

# Python
import math
import random
from collections import deque
import numpy as np

# ros
import rospy
import tf
from active_3d_planning_app_reconstruction.msg import OdometryOffset
from nav_msgs.msg import Odometry


class GPSSimulator(object):
    def __init__(self):
        '''  Initialize ros node and read params '''
        # Parse parameters
        self.position_uncertainty = rospy.get_param('~position_uncertainty',
                                                    0.03)  # [m]
        self.roll_pitch_uncertainty = rospy.get_param(
            '~roll_pitch_uncertainty', 1.0)  # [deg]
        self.yaw_uncertainty = rospy.get_param('~yaw_uncertainty',
                                               3.0)  # [deg]
        self.crop_frequency = rospy.get_param('~crop_frequency',
                                              0.0)  # Hz (0 for max throughput)
        self.noise_model = rospy.get_param(
            '~noise_model',
            "ground_truth")  # ground_truth, uniform, gaussian, random_walk
        self.publish_difference = rospy.get_param('~publish_difference', True)
        self.publish_tf = rospy.get_param('~publish_tf', True)
        # constants
        self.uncertainty = np.array([self.position_uncertainty] * 3 +
                                    [self.roll_pitch_uncertainty] * 2 +
                                    [self.yaw_uncertainty])
        self.measure_length = 20  # For correct (constant) output frequency

        # variables
        self.times = deque([0.0] * self.measure_length, self.measure_length)
        self.previous_time = rospy.Time.now()

        if self.noise_model == "ground_truth":
            self.apply_noise = self.apply_noise_none
        elif self.noise_model == "uniform":
            self.apply_noise = self.apply_noise_uniform
        elif self.noise_model == "gaussian":
            self.apply_noise = self.apply_noise_gaussian
        elif self.noise_model == "random_walk":
            self.apply_noise = self.apply_noise_random_walk
            self.current_offset = np.array([0.0] * 6)
            # Random walk speed in percent of maximum offset per second
            self.walk_speed_range = [
                rospy.get_param('~walk_speed_min', 0.2),
                rospy.get_param('~walk_speed_max', 0.4)
            ]
            self.max_tries = rospy.get_param('~max_tries', 20)
        else:
            rospy.logerr("Unknown GPS noise model '" + self.noise_model + "'!")
            self.apply_noise = self.apply_noise_none

        # Ros
        self.sub = rospy.Subscriber("odometry_in",
                                    Odometry,
                                    self.odom_callback,
                                    queue_size=1,
                                    buff_size=(2**24))
        self.pub = rospy.Publisher("~odometry_out", Odometry, queue_size=10)
        if self.publish_difference:
            self.diff_pub = rospy.Publisher("~difference",
                                            OdometryOffset,
                                            queue_size=10)
        if self.publish_tf:
            self.tf_br = tf.TransformBroadcaster(
            )  # Publish camera transforms in tf

    def odom_callback(self, ros_data):
        ''' Apply some artificial noise to the odom msg and republish. '''

        # Check wether the current frequency is above the desired one
        now = rospy.Time.now()
        if self.crop_frequency > 0.0:
            time_diff = (now - self.previous_time).to_sec()
            if float(self.measure_length) / (np.sum(np.array(self.times)) +
                                             time_diff) > self.crop_frequency:
                return

        # Original pose
        if self.publish_difference:
            prev_ori = np.array([
                ros_data.pose.pose.orientation.x,
                ros_data.pose.pose.orientation.y,
                ros_data.pose.pose.orientation.z,
                ros_data.pose.pose.orientation.w
            ])
            prev_pos = np.array([
                ros_data.pose.pose.position.x, ros_data.pose.pose.position.y,
                ros_data.pose.pose.position.z
            ])

        # Apply noise
        ros_data = self.apply_noise(ros_data)

        # Publish difference
        if self.publish_difference:
            new_ori = np.array([
                ros_data.pose.pose.orientation.x,
                ros_data.pose.pose.orientation.y,
                ros_data.pose.pose.orientation.z,
                ros_data.pose.pose.orientation.w
            ])
            new_pos = np.array([
                ros_data.pose.pose.position.x, ros_data.pose.pose.position.y,
                ros_data.pose.pose.position.z
            ])
            msg = OdometryOffset()
            msg.header.stamp = rospy.Time.now()
            msg.x = (new_pos[0] - prev_pos[0]) * 100  # cm
            msg.y = (new_pos[1] - prev_pos[1]) * 100
            msg.z = (new_pos[2] - prev_pos[2]) * 100
            msg.norm = np.linalg.norm(prev_pos - new_pos) * 100
            (r1, p1, y1) = tf.transformations.euler_from_quaternion(prev_ori)
            (r2, p2, y2) = tf.transformations.euler_from_quaternion(new_ori)
            msg.roll = self.angle_diff(r1, r2) * 180.0 / math.pi  # deg
            msg.pitch = self.angle_diff(p1, p2) * 180.0 / math.pi
            msg.yaw = self.angle_diff(y1, y2) * 180.0 / math.pi
            msg.angle = 2.0 * math.acos(
                min(math.fabs(np.dot(prev_ori, new_ori)), 1.0)) * 180 / math.pi
            self.diff_pub.publish(msg)

        # Publish odometry + transform data
        self.pub.publish(ros_data)

        # tf
        if self.publish_tf:
            self.tf_br.sendTransform(
                (ros_data.pose.pose.position.x, ros_data.pose.pose.position.y,
                 ros_data.pose.pose.position.z),
                (ros_data.pose.pose.orientation.x,
                 ros_data.pose.pose.orientation.y,
                 ros_data.pose.pose.orientation.z,
                 ros_data.pose.pose.orientation.w), ros_data.header.stamp,
                "camera_link", "world")
        # Update time count
        if self.crop_frequency > 0.0:
            self.times.append(time_diff)
        self.previous_time = now

    @staticmethod
    def apply_noise_none(ros_data):
        # Continue with ground truth
        return ros_data

    def apply_noise_uniform(self, ros_data):
        # apply uniform random offsets (jumpy!)
        offset = (np.random.sample(np.shape(self.uncertainty)) -
                  0.5) * 2.0 * self.uncertainty

        orientation = [
            ros_data.pose.pose.orientation.x, ros_data.pose.pose.orientation.y,
            ros_data.pose.pose.orientation.z, ros_data.pose.pose.orientation.w
        ]
        (r, p, y) = tf.transformations.euler_from_quaternion(orientation)

        r = self.add_angle(r, offset[3] / 180.0 * math.pi)
        p = self.add_angle(p, offset[4] / 180.0 * math.pi)
        y = self.add_angle(y, offset[5] / 180.0 * math.pi)

        orientation = tf.transformations.quaternion_from_euler(r, p, y)

        ros_data.pose.pose.position.x = ros_data.pose.pose.position.x + offset[
            0]
        ros_data.pose.pose.position.y = ros_data.pose.pose.position.y + offset[
            1]
        ros_data.pose.pose.position.z = ros_data.pose.pose.position.z + offset[
            2]
        ros_data.pose.pose.orientation.x = orientation[0]
        ros_data.pose.pose.orientation.y = orientation[1]
        ros_data.pose.pose.orientation.z = orientation[2]
        ros_data.pose.pose.orientation.w = orientation[3]
        return ros_data

    def apply_noise_gaussian(self, ros_data):
        # apply gaussian random offsets, bounded by 2x stddev (jumpy!)
        offset = np.random.normal(np.zeros(np.shape(self.uncertainty)),
                                  self.uncertainty)
        offset = np.minimum(offset, self.uncertainty * 2.0)
        offset = np.maximum(offset, self.uncertainty * -2.0)

        orientation = [
            ros_data.pose.pose.orientation.x, ros_data.pose.pose.orientation.y,
            ros_data.pose.pose.orientation.z, ros_data.pose.pose.orientation.w
        ]
        (r, p, y) = tf.transformations.euler_from_quaternion(orientation)

        r = self.add_angle(r, offset[3] / 180.0 * math.pi)
        p = self.add_angle(p, offset[4] / 180.0 * math.pi)
        y = self.add_angle(y, offset[5] / 180.0 * math.pi)

        orientation = tf.transformations.quaternion_from_euler(r, p, y)

        ros_data.pose.pose.position.x = ros_data.pose.pose.position.x + offset[
            0]
        ros_data.pose.pose.position.y = ros_data.pose.pose.position.y + offset[
            1]
        ros_data.pose.pose.position.z = ros_data.pose.pose.position.z + offset[
            2]
        ros_data.pose.pose.orientation.x = orientation[0]
        ros_data.pose.pose.orientation.y = orientation[1]
        ros_data.pose.pose.orientation.z = orientation[2]
        ros_data.pose.pose.orientation.w = orientation[3]
        return ros_data

    def apply_noise_random_walk(self, ros_data):
        # Random walk in 3d, bounded by position_uncertainty radius sphere.
        # Angles walk independently.
        time_diff = (rospy.Time.now() - self.previous_time).to_sec()

        # Random sphere sampling (position)
        count = 0
        while count < self.max_tries:
            distance = random.uniform(self.walk_speed_range[0],
                                      self.walk_speed_range[1]) * time_diff * \
                       self.position_uncertainty
            theta = 2.0 * math.pi * random.random()
            phi = math.acos(1 - 2.0 * random.random()
                            )  # to make points uniformly distributed on sphere
            direction = np.array([
                math.sin(phi) * math.cos(theta),
                math.sin(phi) * math.sin(theta),
                math.cos(phi)
            ])
            if np.linalg.norm(self.current_offset[:3] + direction *
                              distance) <= self.position_uncertainty:
                self.current_offset[:3] = self.current_offset[:3] + direction \
                                          * distance
                break
            else:
                count += 1

        # Roll
        count = 0
        while count < self.max_tries:
            offset = random.uniform(self.walk_speed_range[0],
                                    self.walk_speed_range[1]) * time_diff * \
                     self.roll_pitch_uncertainty
            offset *= (1.0 - 2.0 * random.randrange(2))
            if math.fabs(self.current_offset[3] +
                         offset) < self.roll_pitch_uncertainty:
                self.current_offset[3] = self.current_offset[3] + offset
                break
            else:
                count += 1

        # Pitch
        count = 0
        while count < self.max_tries:
            offset = random.uniform(self.walk_speed_range[0],
                                    self.walk_speed_range[1]
                                    ) * time_diff * self.roll_pitch_uncertainty
            offset *= (1.0 - 2.0 * random.randrange(2))
            if math.fabs(self.current_offset[4] +
                         offset) < self.roll_pitch_uncertainty:
                self.current_offset[4] = self.current_offset[4] + offset
                break
            else:
                count += 1

        # Yaw
        count = 0
        while count < self.max_tries:
            offset = random.uniform(
                self.walk_speed_range[0],
                self.walk_speed_range[1]) * time_diff * self.yaw_uncertainty
            offset *= (1 - 2 * random.randrange(2))
            if math.fabs(self.current_offset[5] +
                         offset) < self.yaw_uncertainty:
                self.current_offset[5] = self.current_offset[5] + offset
                break
            else:
                count += 1

        # Apply noise
        orientation = [
            ros_data.pose.pose.orientation.x, ros_data.pose.pose.orientation.y,
            ros_data.pose.pose.orientation.z, ros_data.pose.pose.orientation.w
        ]
        (r, p, y) = tf.transformations.euler_from_quaternion(orientation)
        r = self.add_angle(r, self.current_offset[3] / 180.0 * math.pi)
        p = self.add_angle(p, self.current_offset[4] / 180.0 * math.pi)
        y = self.add_angle(y, self.current_offset[5] / 180.0 * math.pi)
        orientation = tf.transformations.quaternion_from_euler(r, p, y)
        ros_data.pose.pose.position.x = ros_data.pose.pose.position.x + \
                                        self.current_offset[0]
        ros_data.pose.pose.position.y = ros_data.pose.pose.position.y + \
                                        self.current_offset[1]
        ros_data.pose.pose.position.z = ros_data.pose.pose.position.z + \
                                        self.current_offset[2]
        ros_data.pose.pose.orientation.x = orientation[0]
        ros_data.pose.pose.orientation.y = orientation[1]
        ros_data.pose.pose.orientation.z = orientation[2]
        ros_data.pose.pose.orientation.w = orientation[3]
        return ros_data

    @staticmethod
    def add_angle(angle, to_add):
        """ Return angle scaled to [0, 2pi]"""
        angle += to_add
        if angle > 2.0 * math.pi:
            angle -= 2.0 * math.pi
        elif angle < 0:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def angle_diff(a1, a2):
        angle = (a2 - a1)
        if angle > math.pi:
            angle -= 2.0 * math.pi
        if angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


if __name__ == '__main__':
    rospy.init_node('gps_odometry_simulator', anonymous=True)
    gps = GPSSimulator()
    rospy.spin()
