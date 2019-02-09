#!/usr/bin/env python

# ros
import rospy
from std_srvs.srv import Empty

# Python
import sys
import numpy as np
import datetime
import os
import csv

# Plotting
from matplotlib import pyplot as plt
import Image


class PlottingNode:

    def __init__(self):
        # Parse parameters
        target_dir = rospy.get_param('~target_directory')
        self.method = rospy.get_param('~method', 'single')
        self.ns_voxblox = rospy.get_param('~ns_eval_voxblox_node', '/eval_voxblox_node')
        self.show = rospy.get_param('~show', False)

        # Check for valid params
        methods = {'single': 'single', 'recent': 'recent', 'all': 'all'}  # Dictionary of implemented models
        selected = methods.get(self.method, 'NotFound')
        if selected == 'NotFound':
            warning = "Unknown method '" + model_type_in + "'. Implemented are: " + \
                      "".join(["'" + m + "', " for m in methods])
            rospy.logfatal(warning[:-2])
            sys.exit(-1)
        else:
            self.method = selected

        # Setup
        self.eval_log_file = None
        rospy.wait_for_service(self.ns_voxblox + "/evaluate")
        self.eval_voxblox_srv = rospy.ServiceProxy(self.ns_voxblox + "/evaluate", Empty)

        # Evaluate
        if self.method == 'single':
            self.evaluate(target_dir)
        elif self.method == 'recent':
            subdirs = [o for o in os.listdir(target_dir) if os.path.isdir(os.path.join(target_dir, o))]
            subdirs.sort(reverse=True)
            self.evaluate(os.path.join(target_dir, subdirs[0]))
        elif self.method == 'all':
            subdirs = [o for o in os.listdir(target_dir) if os.path.isdir(os.path.join(target_dir, o))]
            for subdir in subdirs:
                self.evaluate(os.path.join(target_dir, subdir))

        rospy.loginfo("Evaluation completed, shutting down.")

    def evaluate(self, target_dir):
        # Check target dir is valid (approximately)
        if not os.path.isfile(os.path.join(target_dir, "data_log.txt")):
            rospy.logerr("Invalid target directory '%s'.", target_dir)
            return

        # Set params and call the voxblox evaluator
        rospy.set_param(self.ns_voxblox + "/target_directory", target_dir)
        try:
            self.eval_voxblox_srv()
        except:
            rospy.logerr("eval_voxblox service call for '%s' failed .", target_dir)
            return

        # Logfile should be available for voxblox node
        self.eval_log_file = open(os.path.join(target_dir, "data_log.txt"), 'a')
        
        # Read voxblox data file
        data = {}
        headers = None
        with open(os.path.join(target_dir, "voxblox_data.csv")) as infile:
            reader = csv.reader(infile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            for row in reader:
                if row[0] == 'MapName':
                    headers = row
                    for header in headers:
                        data[header] = []
                    continue
                if row[0] != 'Unit':
                    for i in range(len(row)):
                        data[headers[i]].append(row[i])

        # Create Graphs
        rospy.loginfo("Creating Graphs ...")
        if not os.path.isdir(os.path.join(target_dir, "graphs")):
            os.mkdir(os.path.join(target_dir, "graphs"))

        figtype = "SimulationOverview"
        x = np.array(data['RosTime'])
        rmse = np.array(data['RMSE'])
        unknown = np.array(data['UnknownVoxels'])
        truncated = np.array(data['OutsideTruncation'])
        pointclouds = np.cumsum(np.array(data['NPointclouds'], dtype=float))

        _, axes = plt.subplots(4, 1)
        axes[0].plot(x, rmse, 'b-')
        axes[0].set_ylabel('RMSE [m]')
        axes[0].set_ylim(bottom=0)
        axes[1].plot(x, unknown, 'g-')
        axes[1].set_ylabel('Unknown Voxels [%]')
        axes[1].set_ylim(0, 1)
        axes[2].plot(x, truncated, 'r-')
        axes[2].set_ylabel('Truncated Voxels [%]')
        axes[2].set_ylim(0, 1)
        axes[3].plot(x, pointclouds, 'k-')
        axes[3].set_ylabel('Processed Pointclouds [-]')
        axes[3].set_xlabel('Simulated Time [s]')
        plt.suptitle("Simulation Overview")

        save_name = os.path.join(target_dir, "graphs", figtype + ".png")
        if os.path.isfile(save_name):
            os.remove(save_name)
        plt.savefig(save_name, dpi=300, format='png', bbox_inches='tight')
        self.writelog("Created graphs '"+figtype+"'.")
        self.eval_log_file.close()

        if self.show:
            rospy.loginfo("Displaying created graph '%s'. Close to continue...", save_name)
            plt.show()

    def writelog(self, text):
        if self.eval_log_file is not None:
            self.eval_log_file.write(datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S] ") + text + "\n")


if __name__ == '__main__':
    rospy.init_node('eval_plotting_node', anonymous=True)
    pn = PlottingNode()