#define _USE_MATH_DEFINES
#define _POSIX_SOURCE

#include "active_3d_planning/planner/calibration_planner.h"

#include "active_3d_planning/module/trajectory_generator/feasible_rrt_star.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "active_3d_planner");

    // Set logging to debug for testing
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    active_3d_planning::ros::CalibrationPlanner planner_node(nh, nh_private);
    //DO NOT REMOVE THIS! WITHOUT THIS THE LINKER OPTIMIZES AWAY THE MAV LIBRARY AND THUS THE
    //MODULES WON'T BE REGISTERED
    active_3d_planning::trajectory_generator::FeasibleRRTStar test(planner_node);
    ROS_INFO("Initialized active_3d_planner_node.");
    planner_node.planningLoop();
}