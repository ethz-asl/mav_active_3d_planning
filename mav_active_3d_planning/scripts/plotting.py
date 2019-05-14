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
Script to plot all sorts of spontaneous stuff
"""


def plot_gain():
    path = "/home/lukas/Documents/MT/DataFinal/gains"
    dirs = ["gain_unobs", "gain_frontier", "gain_surface_frontier", "gain_weight"]
    data = []
    for d in dirs:
        data.append(read_data(os.path.join(path, d, "orientation_data.csv")))

    # plot args
    plt.rcParams.update({'font.size': 16})
    x = np.arange(4)
    c = ['#ff5050', '#66ff33', '#3399ff', '#cc99ff']       # colors

    # 1 Time
    # y = read_plot(data, 'FinalMeanError', True)
    # bars = plt.bar(x, y[0], yerr=y[1], align='center', alpha=0.8, ecolor='black', capsize=10,
    #                color=(c[0], c[1], c[2], c[3]))
    # # plt.ylabel('Normalized 95% Exploration Time [%]')
    # plt.ylabel('Normalized Final Error Mean [%]')

    #2 legend
    fig = plt.figure()
    plt.bar(0, 0, ecolor='black', color=c[0])
    plt.bar(0, 0, ecolor='black', color=c[1])
    plt.bar(0, 0, ecolor='black', color=c[2])
    plt.bar(0, 0, ecolor='black', color=c[3])
    plt.ylim(top=100)
    fig.set_size_inches(20, 15, forward=True)
    plt.legend(["A: Unobserved Voxels", "B: Frontier Voxels", "C: Surface Frontiers", "D: Our Method"],
               loc='upper center', ncol=2)

    #finish
    # plt.xlim(left=0, right=16)
    plt.xticks(x, ["A", "B", "C", "D"])
    plt.show()


def plot_cost():
    path = "/home/lukas/Documents/MT/DataFinal/costs"
    dirs = ["exp_dist", "exp_time", "rel_dist", "rel_time", "pot_dist", "pot_time", "lin_dist", "lin_time"]
    data = []
    for d in dirs:
        data.append(read_data(os.path.join(path, d, "orientation_data.csv")))

    # plot args
    plt.rcParams.update({'font.size': 16})
    x = np.arange(len(dirs))
    c = ['#ff5050', '#66ff33', '#3399ff', '#cc99ff']       # colors
    h = '//'      # hatches

    # 1 Time
    # y = read_plot(data, 'FinalStdError', True)
    # # 95 none for 0,1, missing for 2, 4 set to 30
    # bars = plt.bar(x, y[0], yerr=y[1], align='center', alpha=0.8, ecolor='black', capsize=10,
    #                color=(c[0], c[0], c[1], c[1], c[2], c[2], c[3], c[3]))
    # bars[1].set_hatch(h)
    # bars[3].set_hatch(h)
    # bars[5].set_hatch(h)
    # bars[7].set_hatch(h)
    # # plt.ylabel('Exploration after 30 minutes [%]')
    # # plt.ylabel('Normalized 95% Exploration Time [%]')
    # plt.ylabel('Normalized Final Error Std [%]')

    # #2 legend
    # fig = plt.figure()
    # plt.bar(0, 0, ecolor='black', color=c[0])
    # plt.bar(0, 0, ecolor='black', color=c[0], hatch=h)
    # plt.bar(0, 0, ecolor='black', color=c[1])
    # plt.bar(0, 0, ecolor='black', color=c[1], hatch=h)
    # plt.bar(0, 0, ecolor='black', color=c[2])
    # plt.bar(0, 0, ecolor='black', color=c[2], hatch=h)
    # plt.bar(0, 0, ecolor='black', color=c[3])
    # plt.bar(0, 0, ecolor='black', color=c[3], hatch=h)
    # plt.ylim(top=100)
    # fig.set_size_inches(25, 15, forward=True)
    # plt.legend(["A: Exponential, Distance", "B: Exponential, Time", "C: Efficiency, Distance", "D: Efficiency, Time",
    #            "E: Discounted, Distance", "F: Discounted, Time", "G: Linear, Distance", "H: Linear, Time"], loc='upper center', ncol=4)

    #finish
    # plt.xlim(left=0, right=16)
    plt.xticks(x, ["A", "B", "C", "D", "E", "F", "G", "H"])
    plt.show()


def plot_sub2():
    path = "/home/lukas/Documents/MT/DataFinal/subsampling"
    x = np.array([1, 2, 3, 4, 5, 8, 10, 15])

    data4 = read_data(os.path.join(path, "sub_test_4", "orientation_data.csv"))
    data8 = read_data(os.path.join(path, "sub_test_8_fail", "orientation_data.csv"))

    # plot args
    plt.rcParams.update({'font.size': 16})

    # Next
    plt.plot(x, data4['NextValMean'], 'b-', linewidth=2)       # NextSeg, NextOri, BestSeg, BestOri
    plt.plot(x, data8['NextValMean'], 'r--', linewidth=2)
    plt.ylabel('Relative Executed Value Loss [%]')
    plt.ylim(bottom=0)

    #finish
    plt.legend(["$n_{ori}=4$", "$n_{ori}=8$"], loc='lower left')
    plt.xlim(left=0, right=16)
    plt.xticks(x, x)
    plt.xlabel("Sub-Sampling Factor $f_{sub}$ [-]")
    plt.show()


def plot_sub():
    path = "/home/lukas/Documents/MT/DataFinal/subsampling"
    x = np.array([1, 2, 3, 4, 5, 8, 10, 15])
    dirs = ["sub_%i" % i for i in x]
    data = []
    for d in dirs:
        data.append(read_data(os.path.join(path, d, "orientation_data.csv")))

    # plot args
    plt.rcParams.update({'font.size': 16})

    # 1 Tree
    # y = read_plot(data, 'TreeSizeMean')
    # plt.errorbar(x, y[0], yerr=y[1], fmt='.b', elinewidth=2)
    # plt.plot(x, y[0], 'b-', linewidth=2)
    # plt.ylabel('Average Trajectory Tree Size [-]')
    # plt.yscale("log")
    # plt.yticks([10, 100, 1000], [10, 100, 1000])

    # 2 Error
    y = read_plot(data, 'FinalMeanError', True)       # FinalStdError
    plt.errorbar(x, y[0], yerr=y[1], fmt='.b', elinewidth=2)
    plt.plot(x, y[0], 'b-', linewidth=2)
    plt.ylabel('Normalized Final Error Mean [%]')
    plt.ylim(bottom=0.5)

    # 3 Time
    # y = read_plot(data, 'T50', True)  # T95
    # plt.errorbar(x, y[0], yerr=y[1], fmt='.b', elinewidth=2)
    # plt.plot(x, y[0], 'b-', linewidth=2)
    # plt.ylabel('Normalized 50% Exploration Time [%]')
    # plt.ylim(bottom=0.5)

    #finish
    plt.xlim(left=0, right=16)
    plt.xticks(x,x)
    plt.xlabel("Sub-Sampling Factor $f_{sub}$ [-]")
    plt.show()


def plot_ori():
    path = "/home/lukas/Documents/MT/DataFinal/orientations"
    x = np.array([1, 2, 4, 8, 16, 32])
    dirs = ["ori_%i" % i for i in x]
    data = []
    for d in dirs:
        data.append(read_data(os.path.join(path, d, "orientation_data.csv")))

    # plot args
    axes_offset_low = 0.1
    axes_offset_up = 4
    plt.rcParams.update({'font.size': 16})

    # 1 Tree
    # y = read_plot(data, 'TreeSizeMean')
    # plt.errorbar(x, y[0], yerr=y[1], fmt='.b', elinewidth=2)
    # plt.plot(x, y[0], 'b-', linewidth=2)
    # plt.ylabel('Average Trajectory Tree Size [-]')
    # plt.yscale("log")
    # plt.yticks([10, 100, 1000], [10, 100, 1000])

    # 2 Error
    # y = read_plot(data, 'FinalStdError', True)       # FinalStdError
    # plt.errorbar(x, y[0], yerr=y[1], fmt='.b', elinewidth=2)
    # plt.plot(x, y[0], 'b-', linewidth=2)
    # plt.ylabel('Normalized Final Error Std [%]')
    # plt.ylim(bottom=0.5)

    # 3 Time
    # y = read_plot(data, 'T95', True)  # T95
    # plt.errorbar(x, y[0], yerr=y[1], fmt='.b', elinewidth=2)
    # plt.plot(x, y[0], 'b-', linewidth=2)
    # plt.ylabel('Normalized 95% Exploration Time [%]')
    # plt.ylim(bottom=0.5)

    # 4 Diff
    y = read_plot(data, 'SecondMean')
    y[0][0] = 1
    y[1][0] = 0
    plt.errorbar(x, y[0], yerr=y[1], fmt='.b', elinewidth=2)
    plt.plot(x, y[0], 'b-', linewidth=2)
    plt.ylabel('Average Relative Gain Difference [%]')
    plt.ylim(bottom=0.0)

    #finish
    plt.xlim(left=x[0] - axes_offset_low, right=x[-1] + axes_offset_up)
    plt.xscale("log", nonposx='clip', basex=2)
    plt.xticks(x, x)
    plt.xlabel("Number of orientations $n_{ori}$ [-]")
    plt.show()


def plot_perception():
    # data
    x = np.linspace(0, 5.0, 200)
    x2 = np.linspace(0, 5, 6)
    y = 0.0024 * np.power(x, 2)
    y2 = 0.0024 * np.power(x2, 2)

    # plot

    plt.rcParams.update({'font.size': 18})
    plt.errorbar(x2, y2, yerr=y2, fmt='.b', elinewidth=2)
    plt.plot(x, y, linewidth=3)
    plt.ylabel('Uncertainty [m]')
    plt.xlim(left=-0.1, right=5.1)
    plt.ylim(top=0.13, bottom=-0.01)

    plt.xlabel("Perceived Depth z [m]")
    plt.show()


def plot_subsampling():
    path = "/home/lukas/Documents/MT/DataFinal/subsampling_4/sub_test_4/orientation_data.csv"
    data = read_data(path)


def read_data(name):
    if not os.path.isfile(name):
        print("Cannot read '%s', skipping file." % name)
        return {}
    data = {}
    headers = None
    with open(name) as infile:
        reader = csv.reader(infile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        for row in reader:
            if not is_num(row[0]) and headers is None:
                headers = row
                for header in headers:
                    data[header] = []
                continue
            else:
                for i in range(len(row)):
                    data[headers[i]].append(row[i])
    return data


def read_plot(data, field, normalize=False):
    means = []
    stddevs = []
    for d in data:
        values = np.array(d[field], dtype=float)
        values = values[~np.isnan(values)]
        values = values[values >= 0]
        if len(values) == 0:
            values = 0
        means.append(np.mean(values))
        stddevs.append(np.std(values))
    means = np.array(means)
    stddevs = np.array(stddevs)
    if normalize:
        maximum = np.max(means)
        means /= maximum
        stddevs /= maximum
    return [means, stddevs]


def is_num(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


if __name__ == '__main__':
    plot_cost()
    print("Plotting finished successfully.")
