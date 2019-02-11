#!/usr/bin/env python

# ros
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool
from voxblox_msgs.srv import FilePath
import tf

# Python
import sys
import math
import numpy as np
import os
import time
import datetime
import csv
import subprocess


class EvalData:

    def __init__(self):
        '''  Initialize ros node and read params '''
        # Parse parameters
        self.ns_planner = rospy.get_param('~ns_planner', "/firefly/planner_node")
        self.planner_delay = rospy.get_param('~delay', 0.0)     # Waiting time until the planner is launched
        self.evaluate = rospy.get_param('~evaluate', False)     # Periodically save the voxblox state

        if self.evaluate:
            # Setup parameters
            self.eval_directory = rospy.get_param('~eval_directory', 'DirParamNotSet')  # Periodically save voxblox map
            if not os.path.isdir(self.eval_directory):
                rospy.logfatal("Invalid target directory '%s'.", self.eval_directory)
                sys.exit(-1)

            self.ns_voxblox = rospy.get_param('~ns_voxblox', "/voxblox/voxblox_node")
            self.eval_frequency = rospy.get_param('~eval_frequency', 5.0)  # Save rate in seconds

            # Statistics
            self.eval_walltime_0 = None
            self.eval_rostime_0 = None
            self.eval_n_maps = 0
            self.eval_n_pointclouds = 0

            # Subscribers
            self.ue_out_sub = rospy.Subscriber("ue_out_in", PointCloud2, self.ue_out_callback, queue_size=10)

            # Setup data directory
            self.eval_directory = os.path.join(self.eval_directory, datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
            os.mkdir(self.eval_directory)
            os.mkdir(os.path.join(self.eval_directory, "voxblox_maps"))
            self.eval_data_file = open(os.path.join(self.eval_directory, "voxblox_data.csv"), 'wb')
            self.eval_writer = csv.writer(self.eval_data_file, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            self.eval_writer.writerow(['MapName', 'RosTime', 'WallTime', 'NPointclouds'])
            self.eval_writer.writerow(['Unit', 'seconds', 'seconds', '-'])
            self.eval_log_file = open(os.path.join(self.eval_directory, "data_log.txt"), 'a')

            # Finish
            self.writelog("Data folder created at '%s'." % self.eval_directory)
            rospy.loginfo("Data folder created at '%s'." % self.eval_directory)
            self.eval_voxblox_service = rospy.ServiceProxy(self.ns_voxblox + "/save_map", FilePath)
            rospy.on_shutdown(self.eval_finish)

        self.launch_simulation()

    def launch_simulation(self):
        rospy.loginfo("Experiment setup: waiting for unreal MAV simulationto setup...")
        # Wait for unreal simulation to setup
        rospy.wait_for_message("unreal_simulation_ready", String)
        rospy.loginfo("Waiting for unreal MAV simulationto setup... done.")

        # Launch planner (by service, every planner needs to advertise this service when ready)
        rospy.loginfo("Waiting for planner to be ready...")
        rospy.wait_for_service(self.ns_planner + "/toggle_running")

        if self.planner_delay > 0:
            rospy.loginfo("Waiting for planner to be ready... done. Launch in %d seconds.", self.planner_delay)
            rospy.sleep(self.planner_delay)
        else:
            rospy.loginfo("Waiting for planner to be ready... done.")
        run_planner_srv = rospy.ServiceProxy(self.ns_planner + "/toggle_running", SetBool)
        run_planner_srv(True)

        # Finish
        rospy.loginfo("Succesfully started the simulation!")
        if self.evaluate:
            self.writelog("Succesfully started the simulation.")

            # Dump complete rosparams for reference
            subprocess.check_call(["rosparam", "dump", os.path.join(self.eval_directory, "rosparams.yaml"), "/"])
            self.writelog("Dumped the parameter server into 'rosparams.yaml'.")

            # Setup first measurements
            self.eval_walltime_0 = time.time()
            self.eval_rostime_0 = rospy.get_time()
            self.eval_n_maps = 0
            self.eval_n_pointclouds = 1

            # Periodic evaluation (call once for initial measurement)
            self.eval_callback(None)
            rospy.Timer(rospy.Duration(self.eval_frequency), self.eval_callback)

    def eval_callback(self, _):
        # Produce a data point
        time_real = time.time() - self.eval_walltime_0
        time_ros = rospy.get_time() - self.eval_rostime_0
        map_name = "{0:05d}".format(self.eval_n_maps)
        self.eval_writer.writerow([map_name, time_ros, time_real, self.eval_n_pointclouds])
        self.eval_voxblox_service(os.path.join(self.eval_directory, "voxblox_maps", map_name + ".vxblx"))
        self.eval_n_pointclouds = 0
        self.eval_n_maps += 1

    def eval_finish(self):
        self.eval_data_file.close()
        map_path = os.path.join(self.eval_directory, "voxblox_maps")
        n_maps = len([f for f in os.listdir(map_path) if os.path.isfile(os.path.join(map_path, f))])
        self.writelog("Finished the simulation, %d/%d maps created." % (n_maps, self.eval_n_maps))
        self.eval_log_file.close()
        rospy.loginfo("On simulation_manager shutdown: closing data files.")

    def writelog(self, text):
        # In case of simulation data being stored, maintain a log file
        if not self.evaluate:
            return
        self.eval_log_file.write(datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S] ") + text + "\n")

    def ue_out_callback(self, _):
        if self.evaluate:
            self.eval_n_pointclouds += 1


if __name__ == '__main__':
    rospy.init_node('eval_data_node', anonymous=True)
    ed = EvalData()
    rospy.spin()

