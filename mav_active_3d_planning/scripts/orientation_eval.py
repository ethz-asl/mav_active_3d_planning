#!/usr/bin/env python

# Python
import sys
import numpy as np
import datetime
import os
import shutil
import csv
import re

# Plotting
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

"""
Script to load and plot data to visualize the impact of the number of orientations while planning. 
"""

# ***** Args *****
p_target_dir = "/home/lukas/Documents/MT/DataFinal/subsampling/sub_test_8"
p_evaluate = True   # True: eval target dir, False: plot full series
p_type = "subsampling"  # base, orientation, subsampling

# Plotting args
p_normalize = False
p_show_plot = True
# ****************


def plot():
    if p_type == "orientation":
        print("Plotting orientations for '%s'." % p_target_dir)
        # Read data
        x = np.array([1, 2, 4, 8, 16, 32])
        dirs = ["ori_%i" % i for i in x]
    elif p_type == "subsampling":
        print("Plotting subsampling for '%s'." % p_target_dir)
        # Read data
        x = np.array([1, 2, 3, 4, 5, 8, 10, 15])
        dirs = ["sub_%i" % i for i in x]

    data = []
    for d in dirs:
        data.append(read_data(os.path.join(p_target_dir, d, "orientation_data.csv")))

    # plot args
    axes_offset_low = 0.1
    axes_offset_up = 4
    # plot
    fig, axes = plt.subplots(3, 2)
    y = read_plot(data, 'TreeSizeMean', p_normalize)
    axes[0, 0].errorbar(x, y[0], yerr=y[1], fmt='.b')
    axes[0, 0].plot(x, y[0], 'b-')
    axes[0, 0].set_xlim(left=x[0] - axes_offset_low, right=x[-1] + axes_offset_up)
    axes[0, 0].set_ylim(bottom=0)
    axes[0, 0].set_ylabel('Average Trajectory Tree Size [-]')
    axes[0, 0].set_xscale("log", nonposx='clip', basex=2)
    axes[0, 0].set_xticks(x)
    axes[0, 0].set_xticklabels(x)

    y = read_plot(data, 'FinalMeanError', p_normalize)
    axes[1, 0].errorbar(x, y[0], yerr=y[1], fmt='.r')
    axes[1, 0].plot(x, y[0], 'r-')
    axes[1, 0].set_xlim(left=x[0] - axes_offset_low, right=x[-1] + axes_offset_up)
    axes[1, 0].set_ylim(bottom=0)
    axes[1, 0].set_ylabel('Final Mean Error [m]')
    axes[1, 0].set_xscale("log", nonposx='clip', basex=2)
    axes[1, 0].set_xticks(x)
    axes[1, 0].set_xticklabels(x)

    y = read_plot(data, 'FinalStdError', p_normalize)
    axes[2, 0].errorbar(x, y[0], yerr=y[1], fmt='.r')
    axes[2, 0].plot(x, y[0], 'r-')
    axes[2, 0].set_xlim(left=x[0] - axes_offset_low, right=x[-1] + axes_offset_up)
    axes[2, 0].set_ylim(bottom=0)
    axes[2, 0].set_ylabel('Final Std Error [m]')
    axes[2, 0].set_xlabel("Number of orientations [-]")
    axes[2, 0].set_xscale("log", nonposx='clip', basex=2)
    axes[2, 0].set_xticks(x)
    axes[2, 0].set_xticklabels(x)

    y = read_plot(data, 'T50', p_normalize)
    y2 = read_plot(data, 'T80', p_normalize)
    y3 = read_plot(data, 'T90', p_normalize)
    y4 = read_plot(data, 'T95', p_normalize)
    y5 = read_plot(data, 'T99', p_normalize)
    axes[0, 1].errorbar(x, y[0], yerr=y[1], fmt='.r')
    axes[0, 1].plot(x, y[0], 'r-')
    axes[0, 1].errorbar(x, y2[0], yerr=y2[1], fmt='.b')
    axes[0, 1].plot(x, y2[0], 'b-')
    axes[0, 1].errorbar(x, y3[0], yerr=y3[1], fmt='.g')
    axes[0, 1].plot(x, y3[0], 'g-')
    axes[0, 1].errorbar(x, y4[0], yerr=y4[1], fmt='.c')
    axes[0, 1].plot(x, y4[0], 'c-')
    axes[0, 1].errorbar(x, y5[0], yerr=y5[1], fmt='.k')
    axes[0, 1].plot(x, y5[0], 'k-')
    axes[0, 1].set_xlim(left=x[0] - axes_offset_low, right=x[-1] + axes_offset_up)
    axes[0, 1].set_ylim(bottom=0)
    axes[0, 1].set_ylabel('Time until X% explored [min]')
    axes[0, 1].legend(["50%", "80%", "90%", "95%", "99%"], loc='lower center', ncol=5, fontsize='small')
    axes[0, 1].set_xscale("log", nonposx='clip', basex=2)
    axes[0, 1].set_xticks(x)
    axes[0, 1].set_xticklabels(x)

    if p_type == "orientation":
        y = read_plot(data, 'SecondMean', p_normalize)
        y[0][0] = 1
        y[1][0] = 0
        axes[1, 1].errorbar(x, y[0], yerr=y[1], fmt='.g')
        axes[1, 1].plot(x, y[0], 'g-')
        axes[1, 1].set_xlim(left=x[0] - axes_offset_low, right=x[-1] + axes_offset_up)
        axes[1, 1].set_ylim(bottom=0)
        axes[1, 1].set_ylabel('Rel. Diff. Best to Second View [%]')
        axes[1, 1].set_xscale("log", nonposx='clip', basex=2)
        axes[1, 1].set_xticks(x)
        axes[1, 1].set_xticklabels(x)
    # elif p_type == "subsampling":
        # y = []
        # for i in range(8):
        # y = read_plot(data, 'SecondMean', p_normalize)
        # axes[1, 1].plot(x, y[0], 'g-')
        # axes[1, 1].set_xlim(left=x[0] - axes_offset_low, right=x[-1] + axes_offset_up)
        # axes[1, 1].set_ylim(bottom=0)
        # axes[1, 1].set_ylabel('Rel. Diff. Best to Second View [%]')
        # axes[1, 1].set_xscale("log", nonposx='clip', basex=2)
        # axes[1, 1].set_xticks(x)
        # axes[1, 1].set_xticklabels(x)

    y = read_plot(data, 'FinalExplored', p_normalize)
    axes[2, 1].errorbar(x, y[0], yerr=y[1], fmt='.k')
    axes[2, 1].plot(x, y[0], 'k-')
    axes[2, 1].set_xlim(left=x[0] - axes_offset_low, right=x[-1] + axes_offset_up)
    axes[2, 1].set_ylim(bottom=0)
    axes[2, 1].set_ylabel('Unexplored points after 30 min [%]')
    axes[2, 1].set_xscale("log", nonposx='clip', basex=2)
    axes[2, 1].set_xticks(x)
    axes[2, 1].set_xticklabels(x)
    axes[2, 1].set_xlabel("Number of orientations [-]")

    if p_normalize:
        plt.suptitle("Orientation Overview (Normalized)")
    else:
        plt.suptitle("Orientation Overview")
    fig.set_size_inches(15, 10, forward=True)

    save_name = os.path.join(p_target_dir, "OrientationOverview.png")
    plt.savefig(save_name, dpi=300, format='png', bbox_inches='tight')
    print("Created Graph: 'OrientationOverview'")
    if p_show_plot:
        plt.show()


def read_plot(data, field, normalize):
    means = []
    stddevs = []
    for d in data:
        values = np.array(d[field], dtype=float)
        values = values[~np.isnan(values)]
        values = values[values >= 0]
        means.append(np.mean(values))
        stddevs.append(np.std(values))
    means = np.array(means)
    stddevs = np.array(stddevs)
    if normalize:
        maximum = np.max(means)
        means /= maximum
        stddevs /= maximum
    return [means, stddevs]


def evaluate():
    # Setup
    print("Evaluating target directory '%s'." % p_target_dir)
    dir_expression = re.compile('\d{8}_\d{6}')   # Only check the default names
    subdirs = [o for o in os.listdir(p_target_dir) if os.path.isdir(os.path.join(p_target_dir, o)) and
               dir_expression.match(o)]

    # Load data
    data_perf = []
    data_test = []
    data_sim = []

    for d in subdirs:
        if p_type == "orientation":
            data_test.append(read_data(os.path.join(p_target_dir, d, "test_data.csv")))
        if p_type == "orientation" or p_type == "base":
            data_perf.append(read_data(os.path.join(p_target_dir, d, "performance_log.csv")))
            data_sim.append(read_data(os.path.join(p_target_dir, d, "voxblox_data.csv")))

    if p_type == "subsampling":
        data_test = read_data(os.path.join(p_target_dir, "test_data.csv"))

    # Write stats to target file
    data_file = open(os.path.join(p_target_dir, "orientation_data.csv"), 'wb')
    data_writer = csv.writer(data_file, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL, lineterminator='\n')

    if p_type == "base" or p_type == "orientation":
        if p_type == "orientation":
            data_writer.writerow(['TreeSizeMean', 'TreeSizeStd', 'T50', 'T80', 'T90', 'T95', 'T99', 'FinalExplored',
                                  'FinalMeanError', 'FinalStdError', 'SecondMean', 'SecondStd'])
        else:
            data_writer.writerow(['TreeSizeMean', 'TreeSizeStd', 'T50', 'T80', 'T90', 'T95', 'T99', 'FinalExplored',
                                  'FinalMeanError', 'FinalStdError'])
        for i in range(len(subdirs)):
            n_traj = np.array(data_perf[i]['NTrajectories'], dtype=float)
            explore = np.array(data_sim[i]['UnknownVoxels'], dtype=float)
            ros_time = np.array(data_sim[i]['RosTime'], dtype=float)
            mean_err = float(data_sim[i]['MeanError'][-1])
            std_err = float(data_sim[i]['StdDevError'][-1])

            quants = [.5, .2, .1, .05, .01]
            quant_values = np.ones_like(quants) * -1
            for t in range(len(explore)):
                for q in range(len(quants)):
                    if quant_values[q] < 0 and explore[t] <= quants[q]:
                        quant_values[q] = ros_time[t] / 60.0

            # Store data
            if p_type == "base":
                data_writer.writerow([np.mean(n_traj), np.std(n_traj), quant_values[0], quant_values[1],
                                      quant_values[2], quant_values[3], quant_values[4], explore[-1], mean_err, std_err])
            else:
                if data_test[i]:
                    ori_best = np.array(data_test[i]['Best'], dtype=float)
                    ori_second = np.array(data_test[i]['Second'], dtype=float)
                    mask = ori_best > 0
                    ori_best = ori_best[mask]
                    ori_second = ori_second[mask]
                    relative_diff = (ori_best - ori_second) / ori_best  # in percent of best
                else:
                    relative_diff = 0
                data_writer.writerow(
                    [np.mean(n_traj), np.std(n_traj), quant_values[0], quant_values[1], quant_values[2],
                     quant_values[3], quant_values[4], explore[-1], mean_err, std_err, np.mean(relative_diff),
                     np.std(relative_diff)])

    elif p_type == "subsampling":
        data_writer.writerow(['ID', 'NextSeg', 'NextOri', 'NextValMean', 'NextValStd', 'BestSeg', 'BestOri',
                              'BestValMean', 'BestValStd', 'NtrajMean', 'NTrajStd', 'NextMean', 'NextStd'])
        next_val_orig = None
        best_val_orig = None
        for i in range(8):
            next_seg = np.array(data_test['NextSeg%i' % i], dtype=float)
            next_ori = np.array(data_test['NextOri%i' % i], dtype=float)
            best_seg = np.array(data_test['BestSeg%i' % i], dtype=float)
            best_ori = np.array(data_test['BestOri%i' % i], dtype=float)
            next_val = np.array(data_test['NextVal%i' % i], dtype=float)
            best_val = np.array(data_test['BestVal%i' % i], dtype=float)
            traj = np.array(data_test['TreeSize'], dtype=float)
            child = np.array(data_test['Children'], dtype=float)

            if i == 0:
                next_val_orig = next_val
                best_val_orig = best_val

            # Exclude values where same segment is chosen
            next_mask = next_ori == 0
            best_mask = best_ori == 0
            next_mask = next_mask & (child > 1)
            best_mask = best_mask & (traj > 2)
            print("Number of decisions (%): ", float(np.sum(child > 1))/len(child))
            child = child[child > 1]

            next_rel = np.divide(next_val, next_val_orig)
            best_rel = np.divide(best_val, best_val_orig)
            next_rel = next_rel[next_mask]
            best_rel = best_rel[best_mask]
            if len(best_rel) == 0:
                best_rel = 1.0
            if len(next_rel) == 0:
                next_rel = 1.0
            data_writer.writerow([i, np.mean(next_seg), np.mean(next_ori), np.mean(next_rel), np.std(next_rel),
                                  np.mean(best_seg), np.mean(best_ori), np.mean(best_rel), np.std(best_rel),
                                  np.mean(traj), np.std(traj), np.mean(child), np.std(child)])

    data_file.close()


def read_data(name):
    if not os.path.isfile(name):
        print("Cannot read '%s', skipping file." % name)
        return []
    data = {}
    headers = None
    with open(name) as infile:
        reader = csv.reader(infile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        for row in reader:
            if row[0] == 'RosTime' or row[0] == 'MapName' or row[0] == 'TreeSizeMean' or row[0] == 'NextSeg0':
                headers = row
                for header in headers:
                    data[header] = []
                continue
            if row[0] != 'Unit':
                for i in range(len(row)):
                    data[headers[i]].append(row[i])
    return data


if __name__ == '__main__':
    if p_evaluate:
        evaluate()
    else:
        plot()
    print("Orientation_eval finished successfully.")
