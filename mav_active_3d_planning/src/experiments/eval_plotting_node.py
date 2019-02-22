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
import re

# Plotting
from matplotlib import pyplot as plt


class EvalPlotting:
    """
    This is the main evaluation node. It expects the data folders and files to have the format hardcoded in the
    simulation_manager and calls the eval_voxblox_node to execute c++ code. Pretty ugly and non-general code but just
    needs to work in this specific case atm...
    """

    def __init__(self):
        # Parse parameters
        target_dir = rospy.get_param('~target_directory')
        self.method = rospy.get_param('~method', 'single')
        self.ns_voxblox = rospy.get_param('~ns_eval_voxblox_node', '/eval_voxblox_node')
        self.evaluate = rospy.get_param('~evaluate', True)
        self.create_plots = rospy.get_param('~create_plots', True)
        self.show_plots = rospy.get_param('~show_plots', False)
        self.create_meshes = rospy.get_param('~create_meshes', True)

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
            self.run_evaluation(target_dir)
        elif self.method == 'recent':
            dir_expression = re.compile('\d{8}_\d{6}') # Only check the default names
            subdirs = [o for o in os.listdir(target_dir) if os.path.isdir(os.path.join(target_dir, o)) and
                       dir_expression.match(o)]
            subdirs.sort(reverse=True)
            if len(subdirs) == 0:
                rospy.loginfo("No recent directories in target dir '%s' to evaluate.", target_dir)
                sys.exit(-1)

            self.run_evaluation(os.path.join(target_dir, subdirs[0]))
        elif self.method == 'all':
            subdirs = [o for o in os.listdir(target_dir) if os.path.isdir(os.path.join(target_dir, o))]
            for subdir in subdirs:
                self.run_evaluation(os.path.join(target_dir, subdir))

        rospy.loginfo("\n" + "*"*53 + "\n* Evaluation completed successfully, shutting down. *\n" + "*"*53)

    def run_evaluation(self, target_dir):
        rospy.loginfo("Starting evaluation on target '%s'.", target_dir)
        # Check target dir is valid (approximately)
        if not os.path.isfile(os.path.join(target_dir, "data_log.txt")):
            rospy.logerr("Invalid target directory.")
            return

        # Check for rosbag renaming
        self.eval_log_file = open(os.path.join(target_dir, "data_log.txt"), 'a+')
        lines = [line.rstrip('\n') for line in self.eval_log_file]
        if not "[FLAG] Rosbag renamed" in lines:
            for line in lines:
                if line[:14] == "[FLAG] Rosbag:":
                    os.rename(os.path.join(os.path.dirname(target_dir), "tmp_bags", line[15:] + ".bag"),
                              os.path.join(target_dir, "visualization.bag"))
                    self.writelog("Moved the tmp rosbag into 'visualization.bag'")
                    self.eval_log_file.write("[FLAG] Rosbag renamed\n")

        self.eval_log_file.close()  # Make it available for voxblox node

        if self.create_meshes:
            # Configure directory
            if not os.path.isdir(os.path.join(target_dir, "meshes")):
                os.mkdir(os.path.join(target_dir, "meshes"))

        if self.evaluate or self.create_meshes:
            # Set params and call the voxblox evaluator
            rospy.set_param(self.ns_voxblox + "/target_directory", target_dir)
            try:
                self.eval_voxblox_srv()
            except:
                rospy.logerr("eval_voxblox service call failed.")
                sys.exit(-1)

        # Reopen
        self.eval_log_file = open(os.path.join(target_dir, "data_log.txt"), 'a')

        if self.create_plots:
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
            meanerr = np.array(data['MeanError'])
            stddev = np.array(data['StdDevError'])
            unknown = np.array(data['UnknownVoxels'])
            truncated = np.array(data['OutsideTruncation'])
            pointclouds = np.cumsum(np.array(data['NPointclouds'], dtype=float))
            cpu_time = np.cumsum(np.array(data['CPUTime'], dtype=float))

            fig, axes = plt.subplots(3, 2)
            axes[0, 0].plot(x, meanerr, 'b-')
            axes[0, 0].set_ylabel('MeanError [m]')
            axes[0, 0].set_ylim(bottom=0)
            axes[1, 0].plot(x, stddev, 'b-')
            axes[1, 0].set_ylabel('StdDevError [m]')
            axes[2, 0].plot(x, truncated, 'r-')
            axes[2, 0].set_ylabel('Truncated Voxels [%]')
            axes[2, 0].set_ylim(0, 1)
            axes[2, 0].set_xlabel('Simulated Time [s]')

            axes[0, 1].plot(x, unknown, 'g-')
            axes[0, 1].set_ylabel('Unknown Voxels [%]')
            axes[0, 1].set_ylim(0, 1)
            axes[1, 1].plot(x, pointclouds, 'k-')
            axes[1, 1].set_ylabel('Processed Pointclouds [-]')
            axes[2, 1].plot(x, cpu_time, 'k-')
            axes[2, 1].set_ylabel('Planner CPU time [s]')
            axes[2, 1].set_xlabel('Simulated Time [s]')
            plt.suptitle("Simulation Overview")
            fig.set_size_inches(15, 10, forward=True)

            save_name = os.path.join(target_dir, "graphs", figtype + ".png")
            if os.path.isfile(save_name):
                os.remove(save_name)
            plt.savefig(save_name, dpi=300, format='png', bbox_inches='tight')
            self.writelog("Created graphs '"+figtype+"'.")
            self.eval_log_file.close()

            if self.show_plots:
                rospy.loginfo("Displaying created graph '%s'. Close to continue...", save_name)
                plt.show()

    def writelog(self, text):
        if self.eval_log_file is not None:
            self.eval_log_file.write(datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S] ") + text + "\n")


if __name__ == '__main__':
    rospy.init_node('eval_plotting_node', anonymous=True)
    ep = EvalPlotting()
