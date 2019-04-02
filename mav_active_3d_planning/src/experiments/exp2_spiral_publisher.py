#!/usr/bin/env python

# ros
import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform
import tf
from std_srvs.srv import SetBool


# Python
import math
import numpy as np


class PathPublisher:

    def __init__(self):
        '''  Initialize ros node and read params '''
        # Parse parameters
        self.loop_time = rospy.get_param('~loop_time', 45.0)    # time per rotation (38~1m/s for 10x14 ellispsis)
        self.x_center = rospy.get_param('~x_center', 5.0)
        self.y_center = rospy.get_param('~y_center', 0.0)
        self.x_width = rospy.get_param('~x_width', 10.0)
        self.y_width = rospy.get_param('~y_width', 15.0)
        self.z_max = rospy.get_param('~z_max', 15.0)
        self.z_step = rospy.get_param('~z_step', 1.5)

        # publisher
        self.traj_pub = rospy.Publisher("command/trajectory", MultiDOFJointTrajectory, queue_size=10)
        self.running = False
        rospy.Service('~toggle_running', SetBool, self.toggleRunning)
        rospy.Service('~get_cpu_time', SetBool, self.getCPUTime)

        while not self.running:
            rospy.sleep(0.1)
        # consts
        sampling_rate = 20.0
        disc = 4

        current_z_step = 0
        z_base = 0.0
        curr_disc = 0.0
        while z_base < self.z_max:
            self.traj_msg = MultiDOFJointTrajectory()
            self.traj_msg.joint_names = ["base_link"]
            t = 0.0
            while t <= self.loop_time/disc:
                t_rel = curr_disc/disc + t / self.loop_time
                transforms = Transform()
                transforms.translation.z = z_base + self.z_step * t_rel
                transforms.translation.x = self.x_center - math.cos(t_rel * 2 * math.pi) * self.x_width / 2
                transforms.translation.y = self.y_center - math.sin(t_rel * 2 * math.pi) * self.y_width / 2
                quaternion = tf.transformations.quaternion_from_euler(0, 0, t_rel * 2.0 * math.pi)
                transforms.rotation.x = quaternion[0]
                transforms.rotation.y = quaternion[1]
                transforms.rotation.z = quaternion[2]
                transforms.rotation.w = quaternion[3]
                point = MultiDOFJointTrajectoryPoint()
                point.time_from_start = rospy.Duration(t)
                point.transforms = [transforms]
                self.traj_msg.points.append(point)
                t = t + 1.0 / sampling_rate

            self.traj_msg.header.stamp = rospy.Time.now()
            self.traj_pub.publish(self.traj_msg)
            rospy.loginfo("publishing ellipse %d/%d, disc %d/%d", current_z_step+1,
                          math.floor(self.z_max/self.z_step)+1, curr_disc+1, disc)
            rospy.sleep(self.loop_time/disc)
            curr_disc = curr_disc + 1
            if curr_disc == disc:
                curr_disc = 0.0
                current_z_step += 1
                z_base = current_z_step * self.z_step
        rospy.loginfo("Finished publishing ellispses")

    def toggleRunning(self, _):
        self.running = True
        return [True, ""]

    def getCPUTime(self, _):
        return [True, "1.0"]


if __name__ == '__main__':
    rospy.init_node('path_publisher', anonymous=True)
    pp = PathPublisher()

