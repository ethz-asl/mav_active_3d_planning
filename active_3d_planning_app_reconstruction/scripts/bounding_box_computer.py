#!/usr/bin/env python

import math
import numpy as np
import rospy
from Tkinter import *
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

"""
Script that computes and visualizes bounding boxes from 3 measured points to facilitate outdoor testing.

To get a bounding box:
- Type 3 measured corner points into the fields
- Hit 'Compute'
- Copy + paste the output into the cfg

Visualize a bounding box in RVIZ:
- Copy + paste a cfg into the output textfield
- Hit 'Visualize'
"""

# ------- Initial Settings -------
bbox_frame = "/world"
bbox_width = 0.1
bbox_color = [0.3, 0.3, 0.8]
point_width = 0.4
point_color = [1, 0.8, 0.2]
initial_pts = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
# --------------------------------

# Init window
window = Tk()
window.title("Bounding Box Computer")
window.geometry('400x170')

# Init ros
rospy.init_node("bounding_box_calculator")
pub = rospy.Publisher("bbox", Marker, queue_size=10)


def compute():
    # read out
    p1 = np.array([float(e_x1.get()), float(e_y1.get()), float(e_z1.get())])
    p2 = np.array([float(e_x2.get()), float(e_y2.get()), float(e_z2.get())])
    p3 = np.array([float(e_x3.get()), float(e_y3.get()), float(e_z3.get())])
    pts = [p1, p2, p3]

    # get corner and edges
    dists = [np.linalg.norm(p[:-1] - p1[:-1]) + np.linalg.norm(p[:-1] - p2[:-1]) + np.linalg.norm(p[:-1] - p3[:-1])
             for p in pts]
    corner = pts[np.argmin(dists)]
    other1 = pts[(np.argmin(dists) + 1) % 3]
    other2 = pts[(np.argmin(dists) + 2) % 3]

    # find rotations
    theta1 = math.degrees(math.atan2(other1[1] - corner[1], other1[0] - corner[0]))
    theta1 = (theta1 + 180) % 90
    theta2 = math.degrees(math.atan2(other2[1] - corner[1], other2[0] - corner[0]))
    theta2 = (theta2 + 180) % 90

    # check average rot
    if math.fabs(theta2 - theta1) <= 45:
        theta = (theta1 + theta2) / 2
        theta_diff = math.fabs(theta2 - theta1)
    else:
        theta = ((theta1 + theta2 + 90) / 2) % 90
        theta_diff = math.min(math.fabs(theta2 - theta1 + 90), math.fabs(theta2 - theta1 - 90))

    # Rotate
    s = math.sin(theta / 180 * math.pi)
    c = math.cos(theta / 180 * math.pi)

    # bounds
    x = [p[0] * c + p[1] * s for p in pts]
    y = [-p[0] * s + p[1] * c for p in pts]
    z = [p[2] for p in pts]

    t_res.delete('1.0', END)
    t_res.insert(END, "x_min: {:.3f}\nx_max: {:.3f}\ny_min: {:.3f}\ny_max: {:.3f}\nz_min: {:.3f}\nz_max: "
                      "{:.3f}\nrotation: {:.3f}".format(np.min(x), np.max(x), np.min(y), np.max(y), np.min(z),
                                                        np.max(z), theta))
    if theta_diff <= 5:
        rospy.loginfo("Success: Bounding Box computed (delta theta %.1fdeg)!" % theta_diff)
    else:
        rospy.logwarn("Large delta theta (%.1fdeg) detected!" % theta_diff)
        rospy.loginfo("Success: Bounding Box computed!")


def visualize():
    text = str(t_res.get("1.0", END)).split("\n")
    fields = ["x_min", "x_max", "y_min", "y_max", "z_min", "z_max", "rotation"]

    # Read data
    data = {}
    for line in text:
        line_found = False
        for field in fields:
            line = line.lstrip(' ')
            if line[0:len(field) + 2] == field + ": ":
                try:
                    value = float(line[len(field) + 2:])
                except:
                    rospy.logwarn("Cannot read value '%s' for field '%s'!" % (line[len(field) + 2:], field))
                    continue
                line_found = True
                data[field] = value
        if not line_found and len(line) > 0:
            rospy.logwarn("Cannot read line '%s'!" % line)

    for field in fields:
        if field not in data:
            data[field] = 0
            rospy.logwarn("Data for field '%s' not set, assuming 0.0 (default)." % field)

    # points
    points = []
    points.append([data["x_max"], data["y_min"], data["z_min"]])
    points.append([data["x_max"], data["y_min"], data["z_max"]])
    points.append([data["x_min"], data["y_min"], data["z_max"]])
    points.append([data["x_min"], data["y_min"], data["z_min"]])
    points.append([data["x_max"], data["y_max"], data["z_min"]])
    points.append([data["x_max"], data["y_max"], data["z_max"]])
    points.append([data["x_min"], data["y_max"], data["z_max"]])
    points.append([data["x_min"], data["y_max"], data["z_min"]])

    # Rotate
    c = math.cos(-data["rotation"] * math.pi / 180)
    s = math.sin(-data["rotation"] * math.pi / 180)
    for i in range(8):
        p = points[i]
        point = Point()
        point.x = c * p[0] + s * p[1]
        point.y = -s * p[0] + c * p[1]
        point.z = p[2]
        points[i] = point

    # Visualize
    marker = Marker()
    marker.type = marker.LINE_LIST
    marker.action = marker.ADD
    marker.ns = "bounding_box"
    marker.header.frame_id = bbox_frame
    marker.header.stamp = rospy.Time.now()
    marker.scale.x = bbox_width
    marker.color.a = 1.0
    marker.color.r = bbox_color[0]
    marker.color.g = bbox_color[1]
    marker.color.b = bbox_color[2]
    marker.points.append(points[0])
    marker.points.append(points[1])
    marker.points.append(points[1])
    marker.points.append(points[2])
    marker.points.append(points[2])
    marker.points.append(points[3])
    marker.points.append(points[3])
    marker.points.append(points[0])
    marker.points.append(points[4])
    marker.points.append(points[5])
    marker.points.append(points[5])
    marker.points.append(points[6])
    marker.points.append(points[6])
    marker.points.append(points[7])
    marker.points.append(points[7])
    marker.points.append(points[4])
    marker.points.append(points[0])
    marker.points.append(points[4])
    marker.points.append(points[1])
    marker.points.append(points[5])
    marker.points.append(points[2])
    marker.points.append(points[6])
    marker.points.append(points[3])
    marker.points.append(points[7])
    pub.publish(marker)

    # triangulation points
    marker = Marker()
    marker.type = marker.SPHERE_LIST
    marker.action = marker.ADD
    marker.ns = "input_points"
    marker.header.frame_id = bbox_frame
    marker.header.stamp = rospy.Time.now()
    marker.scale.x = point_width
    marker.color.a = 1.0
    marker.color.r = point_color[0]
    marker.color.g = point_color[1]
    marker.color.b = point_color[2]
    p = Point()
    p.x = float(e_x1.get())
    p.y = float(e_y1.get())
    p.z = float(e_z1.get())
    marker.points.append(p)
    p = Point()
    p.x = float(e_x2.get())
    p.y = float(e_y2.get())
    p.z = float(e_z2.get())
    marker.points.append(p)
    p = Point()
    p.x = float(e_x3.get())
    p.y = float(e_y3.get())
    p.z = float(e_z3.get())
    marker.points.append(p)
    pub.publish(marker)
    rospy.loginfo("Success: publishing new bounding box visualization!")


# Gui definition
w_x = Label(window, text="x [m]")
w_y = Label(window, text="y [m]")
w_z = Label(window, text="z [m]")
w_1 = Label(window, text="Point 1")
w_2 = Label(window, text="Point 2")
w_3 = Label(window, text="Point 2")
w_bb = Label(window, text="Bounding Box")

w_x.grid(column=0, row=1)
w_y.grid(column=0, row=2)
w_z.grid(column=0, row=3)
w_1.grid(column=1, row=0)
w_2.grid(column=2, row=0)
w_3.grid(column=3, row=0)
w_bb.grid(column=4, row=0)

e_x1 = Entry(window)
e_x2 = Entry(window)
e_x3 = Entry(window)
e_y1 = Entry(window)
e_y2 = Entry(window)
e_y3 = Entry(window)
e_z1 = Entry(window)
e_z2 = Entry(window)
e_z3 = Entry(window)

e_x1.config(width=7)
e_x2.config(width=7)
e_x3.config(width=7)
e_y1.config(width=7)
e_y2.config(width=7)
e_y3.config(width=7)
e_z1.config(width=7)
e_z2.config(width=7)
e_z3.config(width=7)

e_x1.grid(column=1, row=1)
e_x2.grid(column=2, row=1)
e_x3.grid(column=3, row=1)
e_y1.grid(column=1, row=2)
e_y2.grid(column=2, row=2)
e_y3.grid(column=3, row=2)
e_z1.grid(column=1, row=3)
e_z2.grid(column=2, row=3)
e_z3.grid(column=3, row=3)

btn_cmp = Button(window, text="Compute", command=compute, bg="green")
btn_cmp.grid(row=4, column=0, columnspan=4, sticky=W + E + N + S, padx=1, pady=1)

btn_vis = Button(window, text="Visualize", command=visualize, bg="#8080ff")
btn_vis.grid(row=5, column=0, columnspan=4, sticky=W + E + N + S, padx=1, pady=1)

t_res = Text(window, height=8, width=20)
t_res.grid(row=1, column=4, rowspan=5, sticky=W + E + N + S, padx=1, pady=1)

# initial points
e_x1.insert(0, str(initial_pts[0][0]))
e_x2.insert(0, str(initial_pts[1][0]))
e_x3.insert(0, str(initial_pts[2][0]))
e_y1.insert(0, str(initial_pts[0][1]))
e_y2.insert(0, str(initial_pts[1][1]))
e_y3.insert(0, str(initial_pts[2][1]))
e_z1.insert(0, str(initial_pts[0][2]))
e_z2.insert(0, str(initial_pts[1][2]))
e_z3.insert(0, str(initial_pts[2][2]))

# Start
window.mainloop()
