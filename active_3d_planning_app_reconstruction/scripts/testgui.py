#!/usr/bin/env python 
import rospy
from Tkinter import *
from std_srvs.srv import Empty
from std_srvs.srv import SetBool

window = Tk()
window.title("Active Planning Testing GUI")
window.geometry('500x300')

# Set up service calls
pos_hold_service_topic = "/penguin/back_to_position_hold"
toggle_service_topic = "/planner_node/toggle_running"
try:
    rospy.init_node("active_planning_gui")
    rospy.wait_for_service(pos_hold_service_topic, timeout=0.5)
    rospy.wait_for_service(toggle_service_topic, timeout=0.5)
except:
    rospy.logwarn("Services not available")
    exit(1)

pos_hold_service = rospy.ServiceProxy(pos_hold_service_topic,
                                      Empty)

toggle_service = rospy.ServiceProxy(toggle_service_topic,
                                    SetBool)


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


# Gui definition
btn_poshold = Button(window, text="Position Hold", command=poshold)
btn_start = Button(window, text="Start Planner", command=start, bg="green")
btn_stop = Button(window, text="Stop Planner", command=stop, bg="orange")

btn_poshold.config(height=7, width=20)
btn_start.config(height=7, width=20)
btn_stop.config(height=7, width=20)

btn_poshold.grid(column=0, row=0)
btn_start.grid(column=0, row=1)
btn_stop.grid(column=1, row=1)

# Start
window.mainloop()
