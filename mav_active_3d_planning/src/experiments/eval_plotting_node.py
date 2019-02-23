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
        self.show_plots = rospy.get_param('~show_plots', False)     # Auxiliary param, prob removed later
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

        # Create meshes and voxblox eval
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

        # Reopen logfile
        self.eval_log_file = open(os.path.join(target_dir, "data_log.txt"), 'a')

        if self.create_plots:
            # Read voxblox data file
            data_voxblox = {}
            headers = None
            with open(os.path.join(target_dir, "voxblox_data.csv")) as infile:
                reader = csv.reader(infile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                for row in reader:
                    if row[0] == 'MapName':
                        headers = row
                        for header in headers:
                            data_voxblox[header] = []
                        continue
                    if row[0] != 'Unit':
                        for i in range(len(row)):
                            data_voxblox[headers[i]].append(row[i])

            # Read performance data file
            data_perf = {}
            headers = None
            with open(os.path.join(target_dir, "performance_log.csv")) as infile:
                reader = csv.reader(infile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                for row in reader:
                    if row[0] == 'RosTime':
                        headers = row
                        for header in headers:
                            data_perf[header] = []
                        continue
                    for i in range(len(row)):
                        data_perf[headers[i]].append(row[i])

            # Create dirs
            if not os.path.isdir(os.path.join(target_dir, "graphs")):
                os.mkdir(os.path.join(target_dir, "graphs"))

            # Create graphs
            self.plot_sim_overview(data_voxblox, target_dir)
            self.plot_perf_overview(data_perf, target_dir)

            # Finish
            self.writelog("Created graphs 'SimulationOverview', 'PerformanceOverview'.")
            self.eval_log_file.close()

    def plot_sim_overview(self, data, target_dir):
        rospy.loginfo("Creating Graphs: SimulationOverview")

        x = np.array(data['RosTime'])
        meanerr = np.array(data['MeanError'])
        stddev = np.array(data['StdDevError'])
        unknown = np.array(data['UnknownVoxels'])
        truncated = np.array(data['OutsideTruncation'])
        pointclouds = np.cumsum(np.array(data['NPointclouds'], dtype=float))
        wall_time = np.array(data['WallTime'], dtype=float)
        cpu_time = np.array(data['CPUTime'], dtype=float)
        cpu_use =np.zeros(np.shape(cpu_time))
        for i in range(len(cpu_time)-1):
            cpu_use[i] = (cpu_time[i+1])/(wall_time[i+1]-wall_time[i])
        cpu_use[-1] = cpu_use[-2]
        cpu_use = np.repeat(cpu_use, 2)

        fig, axes = plt.subplots(3, 2)
        axes[0, 0].plot(x, meanerr, 'b-')
        axes[0, 0].set_ylabel('MeanError [m]')
        axes[0, 0].set_ylim(bottom=0)
        axes[1, 0].plot(x, stddev, 'b-')
        axes[1, 0].set_ylabel('StdDevError [m]')
        axes[1, 0].set_ylim(bottom=0)
        axes[2, 0].plot(x, truncated, 'r-')
        axes[2, 0].set_ylabel('Truncated Voxels [%]')
        axes[2, 0].set_ylim(0, 1)
        axes[2, 0].set_xlabel('Simulated Time [s]')

        axes[0, 1].plot(x, unknown, 'g-')
        axes[0, 1].set_ylabel('Unknown Voxels [%]')
        axes[0, 1].set_ylim(0, 1)
        axes[1, 1].plot(x, pointclouds, 'k-')
        axes[1, 1].set_ylabel('Processed Pointclouds [-]')

        x = np.repeat(x, 2)
        x = np.concatenate((np.array([0]), x[:-1]))
        axes[2, 1].plot(x, cpu_use, 'k-')
        axes[2, 1].set_ylabel('Average CPU usage [cores]')
        axes[2, 1].set_xlabel('Simulated Time [s]')
        axes[2, 1].set_ylim(bottom=0)
        plt.suptitle("Simulation Overview")
        fig.set_size_inches(15, 10, forward=True)

        save_name = os.path.join(target_dir, "graphs", "SimulationOverview.png")
        plt.savefig(save_name, dpi=300, format='png', bbox_inches='tight')

        if self.show_plots:
            rospy.loginfo("Displaying '%s'. Close to continue...", save_name)
            plt.show()

    def plot_perf_overview(self, data, target_dir):
        rospy.loginfo("Creating Graphs: PerformanceOverview")

        x = np.cumsum(np.array(data['RosTime'], dtype=float))
        y_select = np.array(data['Select'], dtype=float)
        y_expand = np.array(data['Expand'], dtype=float)
        y_gain = np.array(data['Gain'], dtype=float)
        y_cost = np.array(data['Cost'], dtype=float)
        y_value = np.array(data['Value'], dtype=float)
        y_next = np.array(data['NextBest'], dtype=float)
        y_upTG = np.array(data['UpdateTG'], dtype=float)
        y_upTE = np.array(data['UpdateTE'], dtype=float)
        y_vis = np.array(data['Visualization'], dtype=float)
        y_tot = np.array(data['Total'], dtype=float)

        y0 = np.divide(y_select, y_tot)
        y1 = np.divide(y_expand, y_tot) + y0
        y2 = np.divide(y_gain, y_tot) + y1
        y3 = np.divide(y_cost, y_tot) + y2
        y4 = np.divide(y_value, y_tot) + y3
        y5 = np.divide(y_next, y_tot) + y4
        y6 = np.divide(y_upTG, y_tot) + y5
        y7 = np.divide(y_upTE, y_tot) + y6
        y8 = np.divide(y_vis, y_tot) + y7

        sum_tot = np.sum(y_tot)
        s0 = np.sum(y_select)/sum_tot*100
        s1 = np.sum(y_expand)/sum_tot*100
        s2 = np.sum(y_gain)/sum_tot*100
        s3 = np.sum(y_cost)/sum_tot*100
        s4 = np.sum(y_value)/sum_tot*100
        s5 = np.sum(y_next)/sum_tot*100
        s6 = np.sum(y_upTG)/sum_tot*100
        s7 = np.sum(y_upTE)/sum_tot*100
        s8 = np.sum(y_vis)/sum_tot*100
        s9 = 100 - s0 - s1 - s2 - s3 - s4 - s5 - s6 - s7 - s8

        x = np.repeat(x, 2)
        x = np.concatenate((np.array([0]), x[:-1]))
        y0 = np.repeat(y0, 2)
        y1 = np.repeat(y1, 2)
        y2 = np.repeat(y2, 2)
        y3 = np.repeat(y3, 2)
        y4 = np.repeat(y4, 2)
        y5 = np.repeat(y5, 2)
        y6 = np.repeat(y6, 2)
        y7 = np.repeat(y7, 2)
        y8 = np.repeat(y8, 2)

        fig = plt.figure()
        axes = [plt.subplot2grid((5, 1), (0, 0), rowspan=3), plt.subplot(5, 1, 4), plt.subplot(5, 1, 5)]

        axes[0].fill_between(x, 0, y0, facecolor="#a1b400", alpha=.5)
        axes[0].fill_between(x, y0, y1, facecolor="#009000", alpha=.5)
        axes[0].fill_between(x, y1, y2, facecolor="#dc1000", alpha=.5)
        axes[0].fill_between(x, y2, y3, facecolor="#ff4f00", alpha=.5)
        axes[0].fill_between(x, y3, y4, facecolor="#ffb800", alpha=.5)
        axes[0].fill_between(x, y4, y5, facecolor="#ffff00", alpha=.5)
        axes[0].fill_between(x, y5, y6, facecolor="#00eebc", alpha=.5)
        axes[0].fill_between(x, y6, y7, facecolor="#d800dd", alpha=.5)
        axes[0].fill_between(x, y7, y8, facecolor="#a3baff", alpha=.5)
        axes[0].fill_between(x, y8, 1, facecolor="#cccccc", alpha=.5)
        axes[0].set_xlim(left=0, right=x[-1])
        axes[0].set_ylim(bottom=0, top=1)
        axes[0].set_title("Percentage of Time Spent per Function")
        axes[0].set_ylabel('Percent [%]')

        x = np.cumsum(np.array(data['RosTime'], dtype=float))
        n_trajectories = np.array(data['NTrajectories'], dtype=int)
        n_after_update = np.array(data['NTrajAfterUpdate'], dtype=int)
        n_after_update = np.concatenate((np.array([0]), n_after_update[:-1]))
        n_new = n_trajectories - n_after_update

        axes[1].plot(x, n_trajectories, 'b-')
        axes[1].plot(x, n_new, 'g-')
        axes[1].fill_between(x, 0, n_new, facecolor="#009000", alpha=.3)
        axes[1].fill_between(x, n_new, n_trajectories, facecolor="#0000ff", alpha=.3)
        axes[1].set_xlim(left=0, right=x[-1])
        axes[1].set_ylim(bottom=0)
        axes[1].set_title("Trajectory Tree Size")
        axes[1].set_ylabel('TrajectorySegments [-]')
        axes[1].legend(["Total", "New"], loc='upper left', fancybox=True)

        cpu_time = np.array(data['CPU'], dtype=float)
        wall_time = np.array(data['Total'], dtype=float)

        axes[2].plot(x, cpu_time/wall_time, 'k-')
        axes[2].set_xlim(left=0, right=x[-1])
        axes[2].set_ylim(bottom=0)
        axes[2].set_ylabel('CPU Usage [cores]')
        axes[2].set_title("Average CPU Usage")
        axes[2].set_xlabel('Simulated Time [s]')

        fig.set_size_inches(15, 15, forward=True)
        plt.tight_layout()

        box = axes[0].get_position()
        axes[0].set_position([box.x0, box.y0 + box.height * 0.16, box.width, box.height * 0.84])
        legend = ["({0:02.1f}%) Select".format(s0), "({0:02.1f}%) Expand".format(s1), "({0:02.1f}%) Gain".format(s2),
                  "({0:02.1f}%) Cost".format(s3), "({0:02.1f}%) Value".format(s4), "({0:02.1f}%) NextBest".format(s5),
                  "({0:02.1f}%) updateGen".format(s6), "({0:02.1f}%) UpdateEval".format(s7),
                  "({0:02.1f}%) Vis".format(s8), "({0:02.1f}%) Other".format(s9)]
        axes[0].legend(legend, loc='upper center', bbox_to_anchor=(0.5, -0.03), ncol=5, fancybox=True)

        save_name = os.path.join(target_dir, "graphs", "PerformanceOverview.png")
        plt.savefig(save_name, dpi=300, format='png', bbox_inches='tight')

        if self.show_plots:
            rospy.loginfo("Displaying '%s'. Close to continue...", save_name)
            plt.show()

    def writelog(self, text):
        if self.eval_log_file is not None:
            self.eval_log_file.write(datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S] ") + text + "\n")


if __name__ == '__main__':
    rospy.init_node('eval_plotting_node', anonymous=True)
    ep = EvalPlotting()
