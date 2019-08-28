#!/usr/bin/env python

# ros
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf
from std_srvs.srv import SetBool
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform



# Python
import math
import numpy as np
import random


class AEPAdaptor:

    def __init__(self):
        '''  Initialize ros node and read params '''
        # Parse parameters
        self.running = False
        self.position_uncertainty = rospy.get_param('~position_uncertainty', 0.03)  # [m]

        self.pose_pub = rospy.Publisher("~pose_out", PoseStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber("odometry_in", Odometry, self.odom_callback, queue_size=1, buff_size=(2 ** 24))
        rospy.Service('/planner/planner_node/get_cpu_time', SetBool, self.getCPUTime)

        # launch
        rospy.sleep(3.0)
        rospy.Service('/planner/planner_node/toggle_running', SetBool, self.toggleRunning)

    def odom_callback(self, ros_data):
        ''' Transform to pose stamped. '''
        if not self.running:
            return
        msg = PoseStamped()
        msg.header = ros_data.header
        msg.pose = ros_data.pose.pose
        self.pose_pub.publish(msg)


    def toggleRunning(self, _):
        self.running = True
        # Launch
        # traj_msg = MultiDOFJointTrajectory()
        # traj_msg.joint_names = ["base_link"]
        # t = 0.0
        # while t <= 8.0:
        #     transforms = Transform()
        #     transforms.translation.z = 0
        #     transforms.translation.x = 0.25 * t
        #     transforms.translation.y = 0
        #     transforms.rotation.x = 0
        #     transforms.rotation.y = 0
        #     transforms.rotation.z = 0
        #     transforms.rotation.w = 1
        #     point = MultiDOFJointTrajectoryPoint()
        #     point.time_from_start = rospy.Duration(t)
        #     point.transforms = [transforms]
        #     traj_msg.points.append(point)
        #     t = t + 0.02
        # traj_msg.header.stamp = rospy.Time.now()
        # self.traj_pub.publish(traj_msg)
        return [True, ""]

    def getCPUTime(self, _):
        return [True, "1.0"]


if __name__ == '__main__':
    rospy.init_node('aep_active3d_adaptor', anonymous=True)
    adaptor = AEPAdaptor()
    rospy.spin()
