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


def plot_sub2():
    path = "/home/lukas/Documents/MT/DataFinal/subsampling"
    x = np.array([1, 2, 3, 4, 5, 8, 10, 15])

    data4 = read_data(os.path.join(path, "sub_test_4", "orientation_data.csv"))
    data8 = read_data(os.path.join(path, "sub_test_8", "orientation_data.csv"))

    # plot args
    plt.rcParams.update({'font.size': 16})

    # Next
    plt.plot(x, data4['NextValMean'], 'b-', linewidth=2)       # NextSeg, NextOri, BestSeg, BestOri
    plt.plot(x, data8['NextValMean'], 'r--', linewidth=2)
    plt.errorbar(x, np.array(data4['NextValMean'], dtype=float), yerr=np.array(data4['NextValStd'], dtype=float), fmt='.b', elinewidth=2)    # NextValMean, NextValStd, BestValMean, BestValStd
    plt.errorbar(x, np.array(data8['NextValMean'], dtype=float), yerr=np.array(data8['NextValStd'], dtype=float), fmt='.r', elinewidth=2)
    plt.ylabel('Relative Best Value Loss [%]')
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
    # y = read_plot(data, 'FinalStdError', True)       # FinalStdError
    # plt.errorbar(x, y[0], yerr=y[1], fmt='.b', elinewidth=2)
    # plt.plot(x, y[0], 'b-', linewidth=2)
    # plt.ylabel('Normalized Final Error Std [%]')
    # plt.ylim(bottom=0.5)

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
    plot_sub2()
    print("Plotting finished successfully.")
