#include <glog/logging.h>

#include "active_3d_planning_ros/module/module_factory_ros.h"
#include "active_3d_planning_ros/planner/ros_planner.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "active_3d_planner");

  // Set logging to debug for testing
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Debug);
  google::InitGoogleLogging(argv[0]);

  // node handles
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  // Setup
  active_3d_planning::ros::ModuleFactoryROS factory;
  active_3d_planning::Module::ParamMap param_map;
  active_3d_planning::ros::RosPlanner::setupFactoryAndParams(
      &factory, &param_map, nh_private);

  // Build and launch planner
  active_3d_planning::ros::RosPlanner node(nh, nh_private, &factory,
                                           &param_map);
  node.planningLoop();
}
