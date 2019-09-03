#!/usr/bin/env python 
from Tkinter import *
import rospy
from std_srvs.srv import Empty
from std_srvs.srv import SetBool

window = Tk()
window.title("Active Planning Testing GUI")
window.geometry('500x300')

# Set up service calls
pos_hold_service_topic = "/penguin/back_to_position_hold"
toggle_service_topic = "/planner_node/toggle_running"
voxblox_service_topic = "/mav_active_3d_planning/voxblox_node/reset"

try:
    rospy.init_node("active_planning_gui")
    pos_hold_service_topic = rospy.get_param('~pos_hold_service_topic', pos_hold_service_topic)
    toggle_service_topic = rospy.get_param('~toggle_service_topic', toggle_service_topic)
    toggle_service_topic = rospy.get_param('~voxblox_service_topic', voxblox_service_topic)
    rospy.loginfo("Looking for services '%s', '%s', '%s", pos_hold_service_topic, toggle_service_topic,
                  voxblox_service_topic)
    rospy.wait_for_service(pos_hold_service_topic, timeout=0.5)
    rospy.wait_for_service(toggle_service_topic, timeout=0.5)
    rospy.wait_for_service(voxblox_service_topic, timeout=0.5)
except:
    rospy.logwarn("Services not available")
    exit(1)
rospy.loginfo("Services found!")

pos_hold_service = rospy.ServiceProxy(pos_hold_service_topic,  Empty)

toggle_service = rospy.ServiceProxy(toggle_service_topic, SetBool)

voxblox_service = rospy.ServiceProxy(voxblox_service_topic, Empty)


# Button Callbacks
def poshold():
    pos_hold_service()
    rospy.loginfo("Position Hold called")


def start():
    toggle_service(data=True)
    rospy.loginfo("Toggle Service True called")


def stop():
    toggle_service(data=False)
    rospy.loginfo("Toggle Service False called")


def reset_map():
    voxblox_service()
    rospy.loginfo("Reset Voxblox Map called")


# Gui definition
btn_poshold = Button(window, text="Position Hold", command=poshold)
btn_start = Button(window, text="Start Planner", command=start, bg="green")
btn_stop = Button(window, text="Stop Planner", command=stop, bg="orange")
btn_vxblx = Button(window, text="Reset Map", command=reset_map(), bg="blue")

btn_poshold.config(height=7, width=20)
btn_start.config(height=7, width=20)
btn_stop.config(height=7, width=20)
btn_vxblx.config(height=7, width=20)

btn_poshold.grid(column=0, row=0)
btn_start.grid(column=0, row=1)
btn_stop.grid(column=1, row=1)
btn_vxblx.grid(column=1, row=0)

# Start
window.mainloop()
