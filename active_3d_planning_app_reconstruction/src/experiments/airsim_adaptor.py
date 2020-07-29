#!/usr/bin/env python

# ros
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import Pose

# Python
import sys
import time
import csv
import datetime
import os
import re
import subprocess


class AirsimAdaptor(object):

    def __init__(self):
        self.sub = rospy.Subscriber("trajectory_in", MultiDOFJointTrajectory, self.trajectory_conversion_cb, queue_size=10)
        self.pub = rospy.Publisher("command_pose_out", Pose, queue_size=10)

        self.prev_pose = 0

    def trajectory_conversion_cb(self, ros_data):
        if len(ros_data.points) < 1:
            return
        pose = Pose()
        transform = ros_data.points[-1].transforms[-1]
        pose.position.x = transform.translation.x
        pose.position.y = transform.translation.y
        pose.position.z = transform.translation.z
        pose.orientation.x = transform.rotation.x
        pose.orientation.y = transform.rotation.y
        pose.orientation.z = transform.rotation.z
        pose.orientation.w = transform.rotation.w
        self.pub.publish(pose)


if __name__ == '__main__':
    rospy.init_node('airsim_adaptor', anonymous=True)
    aa = AirsimAdaptor()
    rospy.spin()
