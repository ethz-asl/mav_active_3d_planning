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
            if not row[0].isnumeric() and headers is None:
                headers = row
                for header in headers:
                    data[header] = []
                continue
            else:
                for i in range(len(row)):
                    data[headers[i]].append(row[i])
    return data


if __name__ == '__main__':
    plot_perception()
    print("Plotting finished successfully.")
