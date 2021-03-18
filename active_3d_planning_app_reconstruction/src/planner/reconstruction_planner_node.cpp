#include <chrono>
#include <thread>

#include <glog/logging.h>

#include "active_3d_planning_mav/initialization/mav_package.h"
#include "active_3d_planning_ros/module/module_factory_ros.h"
#include "active_3d_planning_ros/planner/ros_planner.h"
#include "active_3d_planning_voxblox/initialization/voxblox_package.h"

int main(int argc, char** argv) {
  // leave some time for the rest to settle
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // init ros
  ros::init(argc, argv, "reconstruction_planner_node");

  // prevent the linker from optimizing these packages away...
  active_3d_planning::initialize::voxblox_package();
  active_3d_planning::initialize::mav_package();

  // Set logging to debug for testing
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Debug);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  // node handles
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  // Setup
  active_3d_planning::ros::ModuleFactoryROS factory;
  active_3d_planning::Module::ParamMap param_map;
  active_3d_planning::ros::RosPlanner::setupFactoryAndParams(
      &factory, &param_map, nh_private);

  // Create and launch the planner
  active_3d_planning::ros::RosPlanner node(nh, nh_private, &factory,
                                           &param_map);
  node.planningLoop();
}
