#!/usr/bin/env python

import csv
import datetime
import os
import re
import shutil
import sys

import numpy as np
import rospy

# Plotting
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from std_srvs.srv import Empty


class EvalPlotting(object):
    """
    This is the main evaluation node. It expects the data folders and files to
    have the format hardcoded in the eval_data_node and calls the
    eval_voxblox_node to execute c++ code. Pretty ugly and non-general code but
    just needs to work in this specific case atm...
    """
    def __init__(self):
        # Parse parameters
        target_dir = rospy.get_param('~target_directory')
        self.method = rospy.get_param('~method', 'single')
        self.ns_voxblox = rospy.get_param('~ns_eval_voxblox_node',
                                          '/eval_voxblox_node')
        self.evaluate = rospy.get_param('~evaluate', True)
        self.evaluate_volume = rospy.get_param('~evaluate_volume', False)
        self.create_plots = rospy.get_param('~create_plots', True)
        self.show_plots = rospy.get_param(
            '~show_plots', False)  # Auxiliary param, prob removed later
        self.create_meshes = rospy.get_param('~create_meshes', True)
        self.series = rospy.get_param(
            '~series', False)  # True: skip single evaluation and create
        # series evaluation data and plots for all runs in the target directory
        self.clear_voxblox_maps = rospy.get_param(
            '~clear_voxblox_maps',
            False)  # rm all maps after eval (disk space!)
        self.unobservable_points_pct = rospy.get_param(
            '~unobservable_points_pct', 0.0)  # Exlude unobservable points
        # from the plots (in percent of total)

        # Check for valid params
        methods = {
            'single': 'single',
            'recent': 'recent',
            'all': 'all'
        }  # Dictionary of implemented models
        selected = methods.get(self.method, 'NotFound')
        if selected == 'NotFound':
            warning = "Unknown method '" + self.method + \
                      "'. Implemented are: " + \
                      "".join(["'" + m + "', " for m in methods])
            rospy.logfatal(warning[:-2])
            sys.exit(-1)
        else:
            self.method = selected

        # Setup
        self.eval_log_file = None
        rospy.wait_for_service(self.ns_voxblox + "/evaluate")
        self.eval_voxblox_srv = rospy.ServiceProxy(
            self.ns_voxblox + "/evaluate", Empty)

        # Evaluate
        if self.series:
            self.evaluate_series(target_dir)
        elif self.method == 'single':
            self.run_single_evaluation(target_dir)
        elif self.method == 'recent':
            dir_expression = re.compile(
                r'\d{8}_\d{6}')  # Only check the default names
            subdirs = [
                o for o in os.listdir(target_dir)
                if os.path.isdir(os.path.join(target_dir, o))
                and dir_expression.match(o)
            ]
            subdirs.sort(reverse=True)
            if len(subdirs) == 0:
                rospy.loginfo(
                    "No recent directories in target dir '%s' to evaluate.",
                    target_dir)
                sys.exit(-1)

            self.run_single_evaluation(os.path.join(target_dir, subdirs[0]))
        elif self.method == 'all':
            subdirs = [
                o for o in os.listdir(target_dir)
                if os.path.isdir(os.path.join(target_dir, o))
            ]
            for subdir in subdirs:
                self.run_single_evaluation(os.path.join(target_dir, subdir))

        rospy.loginfo(
            "\n" + "*" * 53 +
            "\n* Evaluation completed successfully, shutting down. *\n" +
            "*" * 53)

    def run_single_evaluation(self, target_dir):
        rospy.loginfo("Starting evaluation on target '%s'.", target_dir)
        # Check target dir is valid (approximately)
        if not os.path.isfile(os.path.join(target_dir, "data_log.txt")):
            rospy.logerr("Invalid target directory: Could not find a "
                         "'data_log.txt' file.")
            return

        # Check for rosbag renaming
        self.eval_log_file = open(os.path.join(target_dir, "data_log.txt"),
                                  'a+')
        lines = [line.rstrip('\n') for line in self.eval_log_file]
        if not "[FLAG] Rosbag renamed" in lines:
            for line in lines:
                if line[:14] == "[FLAG] Rosbag:":
                    file_name = os.path.join(os.path.dirname(target_dir),
                                             "tmp_bags", line[15:] + ".bag")
                    if os.path.isfile(file_name):
                        os.rename(
                            file_name,
                            os.path.join(target_dir, "visualization.bag"))
                        self.writelog(
                            "Moved the tmp rosbag into 'visualization.bag'")
                        self.eval_log_file.write("[FLAG] Rosbag renamed\n")
                    else:
                        self.writelog("Error: unable to locate '" + file_name +
                                      "'.")
                        rospy.logwarn("Error: unable to locate '" + file_name +
                                      "'.")

        self.eval_log_file.close()  # Make it available for voxblox node

        # Create meshes and voxblox eval
        if self.create_meshes:
            # Configure directory
            if not os.path.isdir(os.path.join(target_dir, "meshes")):
                os.mkdir(os.path.join(target_dir, "meshes"))

        if self.evaluate or self.create_meshes or self.evaluate_volume:
            # Set params and call the voxblox evaluator
            rospy.set_param(self.ns_voxblox + "/target_directory", target_dir)
            try:
                self.eval_voxblox_srv()
            except:
                rospy.logerr(
                    "eval_voxblox service call failed. Shutting down.")
                sys.exit(-1)

        # Reopen logfile
        self.eval_log_file = open(os.path.join(target_dir, "data_log.txt"),
                                  'a+')

        if self.create_plots:
            # Create dirs
            if not os.path.isdir(os.path.join(target_dir, "graphs")):
                os.mkdir(os.path.join(target_dir, "graphs"))

            if os.path.isfile(os.path.join(target_dir, "voxblox_data.csv")):
                # Read voxblox data file
                data_voxblox = self.read_voxblox_data(
                    os.path.join(target_dir, "voxblox_data.csv"))
                if len(data_voxblox['RosTime']) > 1:
                    if 'MeanError' in data_voxblox:
                        self.plot_sim_overview(data_voxblox, target_dir)
                    else:
                        rospy.loginfo(
                            "Unevaluated 'voxblox_data.csv', skipping dependent"
                            " graphs.")
                else:
                    rospy.loginfo(
                        "Too few entries in 'voxblox_data.csv', skipping "
                        "dependent graphs.")
            else:
                rospy.loginfo(
                    "No 'voxblox_data.csv' found, skipping dependent graphs.")

            if os.path.isfile(os.path.join(target_dir, "performance_log.csv")):
                # Read performance data file
                data_perf = {}
                headers = None
                with open(os.path.join(target_dir,
                                       "performance_log.csv")) as infile:
                    reader = csv.reader(infile,
                                        delimiter=',',
                                        quotechar='|',
                                        quoting=csv.QUOTE_MINIMAL)
                    for row in reader:
                        if row[0] == 'RunTime':
                            headers = row
                            for header in headers:
                                data_perf[header] = []
                            continue
                        for i in range(len(row)):
                            data_perf[headers[i]].append(row[i])

                # Create graph
                if len(data_perf['RosTime']) > 1:
                    self.plot_perf_overview(data_perf, target_dir)
                else:
                    rospy.loginfo(
                        "Too few entries in 'performance_log.csv', skipping "
                        "dependent graphs.")
            else:
                rospy.loginfo(
                    "No 'performance_log.csv' found, skipping dependent graphs."
                )

            if os.path.isfile(os.path.join(target_dir, "error_hist.csv")):
                # Read error data file
                with open(os.path.join(target_dir,
                                       "error_hist.csv")) as infile:
                    reader = csv.reader(infile,
                                        delimiter=',',
                                        quotechar='|',
                                        quoting=csv.QUOTE_MINIMAL)
                    data = np.array(list(reader))
                    data_error_hist = data[1:, 1:].astype(int)

                # Create graph
                self.plot_error_hist(data_error_hist, target_dir)
            else:
                rospy.loginfo(
                    "No 'error_hist.csv' found, skipping dependent graphs.")

            # Finish
            if self.clear_voxblox_maps:
                # Remove all voxblox maps to free up disk space
                shutil.rmtree(os.path.join(target_dir, 'voxblox_maps'),
                              ignore_errors=True)
            self.eval_log_file.close()

    def evaluate_series(self, target_dir):
        rospy.loginfo("Evaluating experiment series at '%s'", target_dir)

        # Setup a directory for data, plots, ...
        folder_name = "series_evaluation"
        if not os.path.isdir(os.path.join(target_dir, folder_name)):
            os.mkdir(os.path.join(target_dir, folder_name))
        self.eval_log_file = open(
            os.path.join(target_dir, folder_name, "eval_log.txt"), 'a')

        # Read all the data
        dir_expression = re.compile(r'\d{8}_\d{6}')
        subdirs = [
            o for o in os.listdir(target_dir)
            if os.path.isdir(os.path.join(target_dir, o))
            and dir_expression.match(o)
        ]
        self.writelog("Evaluating '%s' (%i subdirs)." %
                      (target_dir, len(subdirs)))
        voxblox_data = []
        max_data_length = 0
        names = []
        for o in subdirs:
            if os.path.isfile((os.path.join(target_dir, o, "graphs",
                                            "SimulationOverview.png"))):
                # Valid evaluated directory
                data = self.read_voxblox_data(
                    os.path.join(target_dir, o, "voxblox_data.csv"))
                max_data_length = max(max_data_length, len(data["RosTime"]))
                voxblox_data.append(data)
                names.append(o)
            else:
                rospy.logwarn("Experiment at '%s' not properly evaluated!", o)
                self.writelog("Experiment at '%s' not properly evaluated!" % o)

        if max_data_length < 2:
            rospy.loginfo(
                "No valid experiments found, stopping series evaluation.")
            self.writelog(
                "No valid experiments found, stopping series evaluation.")
            self.eval_log_file.close()
            return

        # Create common data timeline by averaging measurement times (these
        # should be similar)
        data_file = open(
            os.path.join(target_dir, folder_name, "series_data.csv"), 'wb')
        data_writer = csv.writer(data_file,
                                 delimiter=',',
                                 quotechar='|',
                                 quoting=csv.QUOTE_MINIMAL,
                                 lineterminator='\n')
        means = {}
        std_devs = {}
        keys = voxblox_data[0].keys()
        keys.remove('RosTime')
        keys = ['RosTime'] + keys  # RosTime is expected as the first argument
        prev_pcls = [0.0] * len(voxblox_data)
        for key in keys:
            means[key] = np.array([])
            std_devs[key] = np.array([])
        for i in range(max_data_length):
            line = []
            if i == 0:
                header_line = []
                for key in keys:
                    header_line.extend((key, ''))
                    for name in names:
                        line.append(name)
                        header_line.append('')
                    line.extend(("Mean", "StdDev"))
                data_writer.writerow(header_line)
                data_writer.writerow(line)
                line = []
            for key in keys:
                values = []
                for dataset in voxblox_data:
                    if i < len(dataset[key]):
                        if key == 'NPointclouds':
                            # These need to accumulate
                            ind = voxblox_data.index(dataset)
                            prev_pcls[ind] = prev_pcls[ind] + float(
                                dataset[key][i])
                            line.append(prev_pcls[ind])
                            values.append(prev_pcls[ind])
                        else:
                            line.append(dataset[key][i])
                            values.append(dataset[key][i])
                    else:
                        line.append("")
                values = np.array(values, dtype=float)
                mean = np.mean(values)
                std = np.std(values)
                means[key] = np.append(means[key], mean)
                std_devs[key] = np.append(std_devs[key], std)
                line.extend((mean, std))
            data_writer.writerow(line)
        data_file.close()

        # Create plot
        rospy.loginfo("Creating graph 'SeriesOverview'")
        x = means['RosTime']
        unit = "s"
        if x[-1] >= 300:
            unit = "min"
            x = np.divide(x, 60)
        cpu_use = np.zeros(np.shape(means['CPUTime']))
        cpu_std = np.zeros(np.shape(means['CPUTime']))
        for i in range(len(means['CPUTime']) - 1):
            div = (means['RosTime'][i + 1] - means['RosTime'][i])
            cpu_use[i] = (means['CPUTime'][i + 1]) / div
            cpu_std[i] = (std_devs['CPUTime'][i + 1]) / div
        cpu_use[-1] = cpu_use[-2]
        cpu_use = np.repeat(cpu_use, 2)
        cpu_std[-1] = cpu_std[-2]
        cpu_std = np.repeat(cpu_std, 2)

        # Plot ends of data series for unequal lengths
        early_stops = []
        x_early = []
        for i in range(len(voxblox_data)):
            dataset = voxblox_data[i]
            length = len(dataset['RosTime']) - 1
            if length < max_data_length - 1:
                early_stops.append(length)
                x_early.append(float(dataset['RosTime'][length]))
                self.writelog("Early stop detected for '%s' at %.2fs." %
                              (names[i], float(dataset['RosTime'][length])))

        fig, axes = plt.subplots(3, 2)
        axes[0, 0].plot(x, means['MeanError'], 'b-')
        axes[0, 0].fill_between(x,
                                means['MeanError'] - std_devs['MeanError'],
                                means['MeanError'] + std_devs['MeanError'],
                                facecolor='b',
                                alpha=.2)
        axes[0, 0].plot([x[i] for i in early_stops],
                        [means['MeanError'][i] for i in early_stops],
                        'kx',
                        markersize=9,
                        markeredgewidth=2)
        axes[0, 0].set_ylabel('MeanError [m]')
        axes[0, 0].set_ylim(bottom=0)
        axes[0, 0].set_xlim(left=0, right=x[-1])
        axes[1, 0].plot(x, means['StdDevError'], 'b-')
        axes[1, 0].fill_between(x,
                                means['StdDevError'] - std_devs['StdDevError'],
                                means['StdDevError'] + std_devs['StdDevError'],
                                facecolor='b',
                                alpha=.2)
        axes[1, 0].plot([x[i] for i in early_stops],
                        [means['StdDevError'][i] for i in early_stops],
                        'kx',
                        markersize=9,
                        markeredgewidth=2)
        axes[1, 0].set_ylabel('StdDevError [m]')
        axes[1, 0].set_ylim(bottom=0)
        axes[1, 0].set_xlim(left=0, right=x[-1])
        axes[2, 0].plot(x, means['OutsideTruncation'], 'r-')
        axes[2, 0].fill_between(
            x,
            means['OutsideTruncation'] - std_devs['OutsideTruncation'],
            means['OutsideTruncation'] + std_devs['OutsideTruncation'],
            facecolor='r',
            alpha=.2)
        axes[2, 0].plot([x[i] for i in early_stops],
                        [means['OutsideTruncation'][i] for i in early_stops],
                        'kx',
                        markersize=9,
                        markeredgewidth=2)
        axes[2, 0].set_ylabel('Truncated Voxels [%]')
        axes[2, 0].set_ylim(0, 1)
        axes[2, 0].set_xlabel("Simulated Time [%s]" % unit)
        axes[2, 0].set_xlim(left=0, right=x[-1])

        # Compensate unobservable voxels
        if np.max(means['UnknownVoxels']) > 0:
            unknown = (means['UnknownVoxels'] - self.unobservable_points_pct
                       ) / (1.0 - self.unobservable_points_pct)
            unknown = np.maximum(unknown, np.zeros_like(unknown))
            axes[0, 1].plot(x, unknown, 'g-')
            axes[0, 1].fill_between(x,
                                    unknown - std_devs['UnknownVoxels'],
                                    unknown + std_devs['UnknownVoxels'],
                                    facecolor='g',
                                    alpha=.2)
            axes[0, 1].plot([x[i] for i in early_stops],
                            [means['UnknownVoxels'][i] for i in early_stops],
                            'kx',
                            markersize=9,
                            markeredgewidth=2)
            axes[0, 1].set_ylabel('Unknown Voxels [%]')
            axes[0, 1].set_ylim(0, 1)
        else:
            axes[0, 1].plot(x, means['Volume'], 'g-')
            axes[0, 1].fill_between(x,
                                    means['Volume'] - std_devs['Volume'],
                                    means['Volume'] + std_devs['Volume'],
                                    facecolor='g',
                                    alpha=.2)
            axes[0, 1].set_ylabel('Explored Volume [m3]')
            axes[0, 1].set_ylim(0, 40 * 40 * 3)
        axes[0, 1].set_xlim(left=0, right=x[-1])
        axes[1, 1].plot(x, means['NPointclouds'], 'k-')
        axes[1,
             1].fill_between(x,
                             means['NPointclouds'] - std_devs['NPointclouds'],
                             means['NPointclouds'] + std_devs['NPointclouds'],
                             facecolor='k',
                             alpha=.2)
        axes[1, 1].plot([x[i] for i in early_stops],
                        [means['NPointclouds'][i] for i in early_stops],
                        'kx',
                        markersize=9,
                        markeredgewidth=2)
        axes[1, 1].set_ylabel('Processed Pointclouds [-]')
        axes[1, 1].set_xlim(left=0, right=x[-1])

        x = np.repeat(x, 2)
        x = np.concatenate((np.array([0]), x[:-1]))
        axes[2, 1].plot(x, cpu_use, 'k-')
        axes[2, 1].fill_between(x,
                                cpu_use - cpu_std,
                                cpu_use + cpu_std,
                                facecolor='k',
                                alpha=.2)
        axes[2, 1].plot([x[i * 2 + 1] for i in early_stops],
                        [cpu_use[i * 2 + 1] for i in early_stops],
                        'kx',
                        markersize=9,
                        markeredgewidth=2)
        axes[2, 1].set_ylabel('Simulated CPU usage [cores]')
        axes[2, 1].set_xlabel("Simulated Time [%s]" % unit)
        axes[2, 1].set_ylim(bottom=0)
        axes[2, 1].set_xlim(left=0, right=x[-1])
        plt.suptitle("Experiment Series Overview (" + str(len(voxblox_data)) +
                     " experiments)\nMeans + Std. Deviations (shaded)")
        fig.set_size_inches(15, 10, forward=True)

        save_name = os.path.join(target_dir, folder_name, "SeriesOverview.png")
        plt.savefig(save_name, dpi=300, format='png', bbox_inches='tight')
        self.writelog("Created graph 'SeriesOverview'.")
        self.eval_log_file.close()

    @staticmethod
    def read_voxblox_data(file_name):
        # Read voxblox data file
        data_voxblox = {}
        headers = None
        with open(file_name) as infile:
            reader = csv.reader(infile,
                                delimiter=',',
                                quotechar='|',
                                quoting=csv.QUOTE_MINIMAL)
            for row in reader:
                if row[0] == 'MapName':
                    headers = row
                    for header in headers:
                        data_voxblox[header] = []
                    continue
                if row[0] != 'Unit':
                    for i in range(len(row)):
                        data_voxblox[headers[i]].append(row[i])
        return data_voxblox

    def plot_sim_overview(self, data, target_dir):
        rospy.loginfo("Creating Graphs: SimulationOverview")
        unit = "s"

        x = np.array(data['RosTime'], dtype=float)
        if x[-1] >= 300:
            unit = "min"
            x = np.divide(x, 60)
        meanerr = np.array(data['MeanError'])
        stddev = np.array(data['StdDevError'])
        truncated = np.array(data['OutsideTruncation'])
        pointclouds = np.cumsum(np.array(data['NPointclouds'], dtype=float))
        ros_time = np.array(data['RosTime'], dtype=float)
        cpu_time = np.array(data['CPUTime'], dtype=float)
        cpu_use = np.zeros(np.shape(cpu_time))
        for i in range(len(cpu_time) - 1):
            cpu_use[i] = (cpu_time[i + 1]) / (ros_time[i + 1] - ros_time[i])
        cpu_use[-1] = cpu_use[-2]
        cpu_use = np.repeat(cpu_use, 2)

        fig, axes = plt.subplots(3, 2)
        axes[0, 0].plot(x, meanerr, 'b-')
        axes[0, 0].set_ylabel('MeanError [m]')
        axes[0, 0].set_ylim(bottom=0)
        axes[0, 0].set_xlim(left=0, right=x[-1])
        axes[1, 0].plot(x, stddev, 'b-')
        axes[1, 0].set_ylabel('StdDevError [m]')
        axes[1, 0].set_ylim(bottom=0)
        axes[1, 0].set_xlim(left=0, right=x[-1])
        axes[2, 0].plot(x, truncated, 'r-')
        axes[2, 0].set_ylabel('Truncated Voxels [%]')
        axes[2, 0].set_ylim(0, 1)
        axes[2, 0].set_xlabel("Simulated Time [%s]" % unit)
        axes[2, 0].set_xlim(left=0, right=x[-1])

        unknown = np.array(data['UnknownVoxels'], dtype=float)
        if np.max(unknown) > 0:
            # compensate unobservable voxels
            unknown = (unknown - self.unobservable_points_pct) / (
                1.0 - self.unobservable_points_pct)  # compensate invisible
            unknown = np.maximum(unknown, np.zeros_like(unknown))
            axes[0, 1].set_ylabel('Unknown Voxels [%]')
            axes[0, 1].set_ylim(0, 1)
        else:
            unknown = np.array(data['Volume'], dtype=float)
            axes[0, 1].set_ylabel('Explored Volume [m3]')
            axes[0, 1].set_ylim(0, 40 * 40 * 3)
        axes[0, 1].plot(x, unknown, 'g-')
        axes[0, 1].set_xlim(left=0, right=x[-1])
        axes[1, 1].plot(x, pointclouds, 'k-')
        axes[1, 1].set_ylabel('Processed Pointclouds [-]')
        axes[1, 1].set_xlim(left=0, right=x[-1])

        x = np.repeat(x, 2)
        x = np.concatenate((np.array([0]), x[:-1]))
        axes[2, 1].plot(x, cpu_use, 'k-')
        axes[2, 1].set_ylabel('Simulated CPU usage [cores]')
        axes[2, 1].set_xlabel("Simulated Time [%s]" % unit)
        axes[2, 1].set_ylim(bottom=0)
        axes[2, 1].set_xlim(left=0, right=x[-1])
        plt.suptitle("Simulation Overview")
        fig.set_size_inches(15, 10, forward=True)

        save_name = os.path.join(target_dir, "graphs",
                                 "SimulationOverview.png")
        plt.savefig(save_name, dpi=300, format='png', bbox_inches='tight')
        self.writelog("Created graph 'SimulationOverview'.")

        if self.show_plots:
            rospy.loginfo("Displaying '%s'. Close to continue...", save_name)
            plt.show()

    def plot_perf_overview(self, data, target_dir):
        rospy.loginfo("Creating Graphs: PerformanceOverview")

        x = np.cumsum(np.array(data['RosTime'], dtype=float))
        unit = "s"
        if x[-1] >= 300:
            unit = "min"
            x = np.true_divide(x, 60)
        y_select = np.array(data['Select'], dtype=float)
        y_expand = np.array(data['Expand'], dtype=float)
        y_gain = np.array(data['Gain'], dtype=float)
        y_cost = np.array(data['Cost'], dtype=float)
        y_value = np.array(data['Value'], dtype=float)
        y_next = np.array(data['NextBest'], dtype=float)
        y_upTG = np.array(data['UpdateTG'], dtype=float)
        y_upTE = np.array(data['UpdateTE'], dtype=float)
        y_vis = np.array(data['Visualization'], dtype=float)
        y_ros = np.array(data['RosCallbacks'], dtype=float)
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
        y9 = 1.0 - np.divide(y_ros, y_tot)

        sum_tot = np.sum(y_tot)
        s0 = np.sum(y_select) / sum_tot * 100
        s1 = np.sum(y_expand) / sum_tot * 100
        s2 = np.sum(y_gain) / sum_tot * 100
        s3 = np.sum(y_cost) / sum_tot * 100
        s4 = np.sum(y_value) / sum_tot * 100
        s5 = np.sum(y_next) / sum_tot * 100
        s6 = np.sum(y_upTG) / sum_tot * 100
        s7 = np.sum(y_upTE) / sum_tot * 100
        s8 = np.sum(y_vis) / sum_tot * 100
        s9 = np.sum(y_ros) / sum_tot * 100
        s10 = 100 - s0 - s1 - s2 - s3 - s4 - s5 - s6 - s7 - s8 - s9

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
        y9 = np.repeat(y9, 2)

        fig = plt.figure()
        axes = [
            plt.subplot2grid((5, 1), (0, 0), rowspan=3),
            plt.subplot(5, 1, 4),
            plt.subplot(5, 1, 5)
        ]

        axes[0].fill_between(x, 0, y0, facecolor="#a1b400", alpha=.5)
        axes[0].fill_between(x, y0, y1, facecolor="#009000", alpha=.5)
        axes[0].fill_between(x, y1, y2, facecolor="#dc1000", alpha=.5)
        axes[0].fill_between(x, y2, y3, facecolor="#ff4f00", alpha=.5)
        axes[0].fill_between(x, y3, y4, facecolor="#ffb800", alpha=.5)
        axes[0].fill_between(x, y4, y5, facecolor="#ffff00", alpha=.5)
        axes[0].fill_between(x, y5, y6, facecolor="#00eebc", alpha=.5)
        axes[0].fill_between(x, y6, y7, facecolor="#d800dd", alpha=.5)
        axes[0].fill_between(x, y7, y8, facecolor="#a3baff", alpha=.5)
        axes[0].fill_between(x, y8, y9, facecolor="#cccccc", alpha=.5)
        axes[0].fill_between(x, y9, 1, facecolor="#606060", alpha=.5)
        axes[0].set_xlim(left=0, right=x[-1])
        axes[0].set_ylim(bottom=0, top=1)
        axes[0].set_title("Percentage of CPU Time Spent per Function")
        axes[0].set_ylabel('Percent [%]')

        x = np.cumsum(np.array(data['RosTime'], dtype=float))
        if unit == "min":
            x = np.true_divide(x, 60)
        n_trajectories = np.array(data['NTrajectories'], dtype=int)
        n_after_update = np.array(data['NTrajAfterUpdate'], dtype=int)
        n_after_update = np.concatenate((np.array([0]), n_after_update[:-1]))
        n_new = n_trajectories - n_after_update

        axes[1].plot(x, n_trajectories, 'b-')
        axes[1].plot(x, n_new, 'g-')
        axes[1].fill_between(x, 0, n_new, facecolor="#009000", alpha=.3)
        axes[1].fill_between(x,
                             n_new,
                             n_trajectories,
                             facecolor="#0000ff",
                             alpha=.3)
        axes[1].set_xlim(left=0, right=x[-1])
        axes[1].set_ylim(bottom=0)
        axes[1].set_title("Trajectory Tree Size")
        axes[1].set_ylabel('TrajectorySegments [-]')
        axes[1].legend(["Total", "New"], loc='upper left', fancybox=True)

        x = np.array([])
        ros_time = np.array(data['RosTime'], dtype=float)
        cpu_times = [
            np.array(data['Total'], dtype=float), y_select + y_expand +
            y_gain + y_cost + y_value + y_next + y_upTE + y_upTG
        ]  # Total, Planning
        cpu_use = [np.array([])] * len(cpu_times)
        i = 0
        averaging_threshold = 2.0  # seconds, for smoothing
        t_curr = ros_time[0]
        x_curr = 0
        cpu_curr = [time[0] for time in cpu_times]
        while i + 1 < len(ros_time):
            i = i + 1
            if t_curr >= averaging_threshold:
                for j in range(len(cpu_times)):
                    cpu_use[j] = np.append(cpu_use[j], cpu_curr[j] / t_curr)
                x_curr = x_curr + t_curr
                x = np.append(x, x_curr)
                t_curr = ros_time[i]
                for j in range(len(cpu_times)):
                    cpu_curr[j] = cpu_times[j][i]
            else:
                t_curr = t_curr + ros_time[i]
                for j in range(len(cpu_times)):
                    cpu_curr[j] = cpu_curr[j] + cpu_times[j][i]

        if unit == "min":
            x = np.true_divide(x, 60)

        axes[2].plot(x, cpu_use[0], 'k-')
        axes[2].plot(x, cpu_use[1], linestyle='-', color='#5492E7')
        axes[2].plot(np.array([0, x[-1]]),
                     np.array([1, 1]),
                     linestyle='-',
                     color='0.7',
                     alpha=0.8)
        axes[2].set_xlim(left=0, right=x[-1])
        axes[2].set_ylim(bottom=0)
        axes[2].set_ylabel('CPU Usage [cores]')
        axes[2].set_title("Planner Consumed CPU Time per Simulated Time")
        axes[2].set_xlabel('Simulated Time [%s]' % unit)
        axes[2].legend(["Process", "Planning"],
                       loc='upper left',
                       fancybox=True)

        fig.set_size_inches(15, 15, forward=True)
        plt.tight_layout()

        box = axes[0].get_position()
        axes[0].set_position(
            [box.x0, box.y0 + box.height * 0.16, box.width, box.height * 0.84])
        legend = [
            "({0:02.1f}%) Select".format(s0), "({0:02.1f}%) Expand".format(s1),
            "({0:02.1f}%) Gain".format(s2), "({0:02.1f}%) Cost".format(s3),
            "({0:02.1f}%) Value".format(s4),
            "({0:02.1f}%) NextBest".format(s5),
            "({0:02.1f}%) updateGen".format(s6),
            "({0:02.1f}%) UpdateEval".format(s7),
            "({0:02.1f}%) Vis".format(s8), "({0:02.1f}%) Other".format(s10),
            "({0:02.1f}%) ROS".format(s9)
        ]
        axes[0].legend(legend,
                       loc='upper center',
                       bbox_to_anchor=(0.5, -0.04),
                       ncol=6,
                       fancybox=True)

        save_name = os.path.join(target_dir, "graphs",
                                 "PerformanceOverview.png")
        plt.savefig(save_name, dpi=300, format='png', bbox_inches='tight')
        self.writelog("Created graph 'PerformanceOverview'.")

        if self.show_plots:
            rospy.loginfo("Displaying '%s'. Close to continue...", save_name)
            plt.show()

    def plot_error_hist(self, data, target_dir):
        rospy.loginfo("Creating Graphs: ErrorHistogram")

        time = np.arange(np.shape(data)[0])
        bins = np.arange(np.shape(data)[1])
        B, T = np.meshgrid(bins, time)

        # Plotting
        fig = plt.figure()
        ax = fig.add_subplot(211, projection='3d')
        Xi = B.flatten()
        Yi = T.flatten()
        Zi = np.zeros(data.size)

        dx = .5 * np.ones(data.size)
        dy = .5 * np.ones(data.size)
        dz = data.flatten()

        ax.set_xlabel('error bins [0, trunc]')
        ax.set_ylabel('time [measurements]')
        ax.set_zlabel('bin count')
        ax.bar3d(Xi, Yi, Zi, dx, dy, dz, color='g')
        ax.set_title('Absolute error histogram')

        ax = fig.add_subplot(212, projection='3d')
        data2 = data.astype(float)
        for i in range(np.shape(data)[0]):
            data2[i, :] = data2[i, :] / np.sum(data2[i, :])

        dz = data2.flatten()
        ax.set_xlabel('error bins [0, trunc]')
        ax.set_ylabel('time [measurements]')
        ax.set_zlabel('bin percentage')
        ax.bar3d(Xi, Yi, Zi, dx, dy, dz, color='b')
        ax.set_title('Relative error histogram')
        fig.set_size_inches(10, 12, forward=True)
        plt.tight_layout()

        save_name = os.path.join(target_dir, "graphs", "ErrorHistogram.png")
        plt.savefig(save_name, dpi=300, format='png', bbox_inches='tight')
        self.writelog("Created graph 'ErrorHistogram'.")

        if self.show_plots:
            rospy.loginfo("Displaying '%s'. Close to continue...", save_name)
            plt.show()

    def writelog(self, text):
        if self.eval_log_file is not None:
            self.eval_log_file.write(
                datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S] ") +
                text + "\n")


if __name__ == '__main__':
    rospy.init_node('eval_plotting_node', anonymous=True)
    ep = EvalPlotting()
