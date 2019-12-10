#!/usr/bin/env python

# ros
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_py as tf2
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

# Python
import sys
import math
import numpy as np
import time


class Trafo:

    def __init__(self):
        '''  Initialize ros node and read params '''
        self.target_frame = rospy.get_param('~target_frame', "world")
        self.timeout = rospy.get_param('~timeout', 0.0)

        # Initialize node
        self.pub = rospy.Publisher("~pcl_out", PointCloud2, queue_size=10)
        self.sub = rospy.Subscriber("~pcl_in", PointCloud2, self.callback, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def callback(self, msg):
        try:
            print("A")
            trans = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id,
                                               msg.header.stamp) #, rospy.Duration(self.timeout))
            print("B")
        except tf2.LookupException as ex:
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(ex)
            return
        cloud_out = do_transform_cloud(msg, trans)
        self.pub.publish(cloud_out)


if __name__ == '__main__':
    rospy.init_node('pcl_transformer', anonymous=True)
    trafo = Trafo()
    rospy.spin()


