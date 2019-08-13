#define _USE_MATH_DEFINES
#define _POSIX_SOURCE

#include "active_3d_planning/planner/ros_planner.h"

#include <iostream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "active_3d_planner");

    // Set logging to debug for testing
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    active_3d_planning::ros::RosPlanner planner_node(nh, nh_private);
    ROS_INFO("Initialized active_3d_planner_node.");
    planner_node.planningLoop();
}