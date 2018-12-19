#!/usr/bin/env python

# ros
import rospy
from std_srvs.srv import Empty, SetBool
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray
import sensor_msgs.point_cloud2 as pc2

# Python
import sys
import math
import numpy as np
import time
from collections import deque

# Plotting
from matplotlib import pyplot as plt


class ProgressMonitor:

    def __init__(self):
        '''  Initialize ros node and read params '''
        # Parse parameters
        self.regulate = rospy.get_param('~regulate', False)     # Manage odom throughput for unreal_ros_client
        self.voxel_size = rospy.get_param('~voxel_size', 0.2)  # Manage odom throughput for unreal_ros_client

        self.d_voxels_occ = np.array([])
        self.d_voxels_free = np.array([])
        self.d_tsdf_time = np.array([])
        self.d_voxels_surf = np.array([])
        self.d_surf_time = np.array([])

        # Subscribers
        self.tsdf_sub = rospy.Subscriber("tsdf_in", PointCloud2, self.tsdf_callback, queue_size=10)
        self.surf_sub = rospy.Subscriber("surface_pcl_in", PointCloud2, self.surf_callback, queue_size=10)

        # self.pub = rospy.Publisher("~data", Float64MultiArray, queue_size=10)
        rospy.on_shutdown(self.save_plot)

        # Printing service
        # rospy.Service('~display_monitor', Empty, self.mon_print_handle)

    def tsdf_callback(self, ros_data):
        points = np.array([p for p in pc2.read_points(ros_data, skip_nans=True, field_names=("x", "y", "z",
                                                                                             "intensity"))])
        if np.size(points, 0) == 0:
            return

        self.d_tsdf_time = np.append(self.d_tsdf_time, rospy.Time.now().to_sec())
        self.d_voxels_free = np.append(self.d_voxels_free, np.sum(points[:, 3] > 0)*self.voxel_size**3)
        self.d_voxels_occ = np.append(self.d_voxels_occ, (np.size(points, 0)-self.d_voxels_free[-1])*self.voxel_size**3)

    def surf_callback(self, ros_data):
        points = np.array([p for p in pc2.read_points(ros_data, skip_nans=True, field_names=("x", "y", "z"))])

        if np.size(points, 0) == 0:
            return

        self.d_surf_time = np.append(self.d_surf_time, rospy.Time.now().to_sec())
        self.d_voxels_surf = np.append(self.d_voxels_surf, np.size(points, 0)*self.voxel_size**3)

    def save_plot(self):
        t = self.d_tsdf_time
        plt.plot(t, self.d_voxels_free, 'b-', t, self.d_voxels_occ, 'r--', self.d_surf_time, self.d_voxels_surf, 'g-.',
                 t, self.d_voxels_occ + self.d_voxels_free, 'k:')
        plt.ylabel("Volume [m^3]")
        plt.xlabel('Simulated Time [s]')
        plt.legend(labels=["Free Voxels", "Occupied Voxels", "Surface out of Occ.", "Total Voxels"], loc=2)
        plt.savefig("/home/lukas/Pictures/uecvros/"+str(int(time.time())), dpi=300, format='png', bbox_inches='tight')


if __name__ == '__main__':
    rospy.init_node('progress_monitor', anonymous=True)
    pm = ProgressMonitor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down progress_monitor")
