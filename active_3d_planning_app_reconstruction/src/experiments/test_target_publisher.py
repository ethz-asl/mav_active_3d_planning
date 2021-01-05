#!/usr/bin/env python

# ros
import rospy
from geometry_msgs.msg import Pose, Transform, Twist
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from std_srvs.srv import Empty
import tf

# Python
import numpy as np
import random
import math


class TargetPublisher(object):

    def __init__(self):
        self.sub = rospy.Subscriber("odometry_in", Odometry, self.odom_cb, queue_size=10)
        self.pub = rospy.Publisher("command_pose_out", Pose, queue_size=10)
        self.dist_thresh = 0.2  # m
        #self.rot_thresh = 3  # deg
        self.traj_length = 2  # m
        self.ns_gazebo = rospy.get_param('~ns_gazebo', "/gazebo")
        self.ns_mav = rospy.get_param('~ns_mav', "/firefly")
        self.initial_position = rospy.get_param('~initial_position', [0, 0, 0])  # x, y, z [m]

        self.goal_pos = None
        self.launched = False
        self.launch()

    def launch(self):
        # Wait for Gazebo services
        rospy.loginfo("Starting unreal MAV simulation setup coordination...")
        rospy.wait_for_service(self.ns_gazebo + "/unpause_physics")
        rospy.wait_for_service(self.ns_gazebo + "/set_model_state")

        # Prepare initialization trajectory command
        traj_pub = rospy.Publisher(self.ns_mav + "/command/trajectory",
                                   MultiDOFJointTrajectory, queue_size=10)
        traj_msg = MultiDOFJointTrajectory()
        traj_msg.joint_names = ["base_link"]
        transforms = Transform()
        transforms.translation.x = self.initial_position[0]
        transforms.translation.y = self.initial_position[1]
        transforms.translation.z = self.initial_position[2]
        point = MultiDOFJointTrajectoryPoint([transforms], [Twist()], [Twist()],
                                             rospy.Duration(0))
        traj_msg.points.append(point)

        # Prepare initialization Get/SetModelState service
        set_model_srv = rospy.ServiceProxy(self.ns_gazebo + "/set_model_state",
                                           SetModelState)
        get_model_srv = rospy.ServiceProxy(self.ns_gazebo + "/get_model_state",
                                           GetModelState)
        mav_name = self.ns_mav[np.max(
            [i for i in range(len(self.ns_mav)) if self.ns_mav[i] == "/"]) + 1:]
        pose = Pose()
        pose.position.x = self.initial_position[0]
        pose.position.y = self.initial_position[1]
        pose.position.z = self.initial_position[2]
        model_state_set = ModelState(mav_name, pose, Twist(), "world")

        # Wake up gazebo
        rospy.loginfo("Waiting for gazebo to wake up ...")
        unpause_srv = rospy.ServiceProxy(self.ns_gazebo + "/unpause_physics",
                                         Empty)
        while not unpause_srv():
            rospy.sleep(0.1)
        rospy.loginfo("Waiting for gazebo to wake up ... done.")

        # Wait for drone to spawn (imu is publishing)
        rospy.loginfo("Waiting for MAV to spawn ...")
        rospy.wait_for_message(self.ns_mav + "/imu", Imu)
        rospy.loginfo("Waiting for MAV to spawn ... done.")

        # Initialize drone stable at [0, 0, 0]
        rospy.loginfo("Waiting for MAV to stabilize ...")
        dist = 10  # Position and velocity
        while dist >= 0.1:
            traj_msg.header.stamp = rospy.Time.now()
            traj_pub.publish(traj_msg)
            set_model_srv(model_state_set)
            rospy.sleep(0.1)
            state = get_model_srv(mav_name, "world")
            pos = state.pose.position
            twist = state.twist.linear
            dist = np.sqrt((pos.x - self.initial_position[0]) ** 2 + (
                        pos.y - self.initial_position[1]) ** 2 +
                           (pos.z - self.initial_position[2]) ** 2) + np.sqrt(
                twist.x ** 2 + twist.y ** 2 + twist.z ** 2)
        rospy.loginfo("Waiting for MAV to stabilize ... done.")
        rospy.loginfo("Launched the simulation.")
        self.launched = True

    def odom_cb(self, odom):
        if not self.launched:
            return
        if self.goal_pos is None:
            self.goal_pos = np.array([odom.pose.pose.position.x,
                                      odom.pose.pose.position.y,
                                      odom.pose.pose.position.z])

        # Check distance
        pos = np.array([odom.pose.pose.position.x,
                        odom.pose.pose.position.y,
                        odom.pose.pose.position.z])
        if np.linalg.norm(pos-self.goal_pos) <= self.dist_thresh:
            quaternion = tf.transformations.quaternion_from_euler(0, 0, random.random() * 2 * math.pi)

            pose = Pose()
            pose.position.x = pos[0] + (random.random() -0.5) * 2 * self.traj_length
            pose.position.y = pos[1] + (random.random() -0.5) * 2 * self.traj_length
            pose.position.z = pos[2] + (random.random() -0.5) * 2 * self.traj_length
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            self.pub.publish(pose)
            self.goal_pos = np.array([pose.position.x,
                                      pose.position.y,
                                      pose.position.z])



if __name__ == '__main__':
    rospy.init_node('target_publisher', anonymous=True)
    tp = TargetPublisher()
    rospy.spin()
