#define _USE_MATH_DEFINES
#define _POSIX_SOURCE

#include "mav_active_3d_planning/planner_node_class.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/conversions.h>

#include <random>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "active_3d_planner");

    // Set logging to debug for testing
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    mav_active_3d_planning::PlannerNode planner_node(nh, nh_private);
    ROS_INFO("Initialized active_3d_planner_node.");
    planner_node.planningLoop();
}