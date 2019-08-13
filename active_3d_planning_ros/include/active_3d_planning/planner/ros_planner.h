#ifndef ACTIVE_3D_PLANNING_ROS_ROS_PLANNER_H_
#define ACTIVE_3D_PLANNING_ROS_ROS_PLANNER_H_

#include "active_3d_planning/module/back_tracker.h"
#include "active_3d_planning/module/module_factory_ros.h"
#include "active_3d_planning/module/trajectory_evaluator.h"
#include "active_3d_planning/module/trajectory_generator.h"
#include "active_3d_planning/data/trajectory_segment.h"
#include "active_3d_planning/planner/planner_I.h"
#include "active_3d_planning/tools/defaults.h"

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox_ros/esdf_server.h>

#include <ctime>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace active_3d_planning {
namespace ros {

class RosPlanner : public PlannerI {
public:
  RosPlanner(const ::ros::NodeHandle &nh, const ::ros::NodeHandle &nh_private);

  virtual ~RosPlanner() = default;

  // ros callbacks
  void odomCallback(const nav_msgs::Odometry &msg);

  bool runSrvCallback(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res);

  bool cpuSrvCallback(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res);

  // Start/Run the planning loop
  void planningLoop();

  // Accessors
  const Eigen::Vector3d &getCurrentPosition() const override {
    return current_position_;
  }
  const Eigen::Quaterniond &getCurrentOrientation() const override {
    return current_orientation_;
  }

  virtual BackTracker &getBackTracker() override { return *back_tracker_; }
  virtual TrajectoryGenerator &getTrajectoryGenerator() override {
    return *trajectory_generator_;
  }
  virtual TrajectoryEvaluator &getTrajectoryEvaluator() override {
    return *trajectory_evaluator_;
  }

  virtual ModuleFactory &getFactory() override { return factory_; }

  // maybe want to get rid of voxblox at some point
  virtual voxblox::EsdfServer &getMap() override { return *voxblox_server_; }

protected:
  // factory
  ModuleFactoryROS factory_;
  // ros
  ::ros::NodeHandle nh_;
  ::ros::NodeHandle nh_private_;
  ::ros::Subscriber odom_sub_;
  ::ros::Publisher target_pub_;
  ::ros::Publisher trajectory_vis_pub_;
  ::ros::Publisher target_reached_pub_;
  ::ros::ServiceServer run_srv_;
  ::ros::ServiceServer get_cpu_time_srv_;

  // members
  std::shared_ptr<voxblox::EsdfServer> voxblox_server_;
  std::unique_ptr<TrajectoryGenerator> trajectory_generator_;
  std::unique_ptr<TrajectoryEvaluator> trajectory_evaluator_;
  std::unique_ptr<BackTracker> back_tracker_;

  // variables
  std::unique_ptr<TrajectorySegment>
      current_segment_;             // root node of full trajectory tree
  bool running_;                    // whether to run the main loop
  Eigen::Vector3d target_position_; // current movement goal
  double target_yaw_;
  Eigen::Vector3d current_position_; // Current pose of the mav
  Eigen::Quaterniond current_orientation_;
  bool target_reached_; // whether the goal point was reached, update based on
                        // odom input
  int vis_num_previous_trajectories_; // Counter for visualization function
  int vis_num_previous_evaluations_;
  int vis_completed_count_;
  int new_segments_; // keep track of min/max tries and segments
  int new_segment_tries_;
  bool min_new_value_reached_;

  // Info+performance bookkeeping
  ::ros::Time info_timing_;       // Rostime for verbose and perf
  int info_count_;              // num trajectories counter for verbose
  int info_killed_next_;        // number of segments killed during root change
  int info_killed_update_;      // number of segments killed during updating
  std::ofstream perf_log_file_; // performance file
  std::vector<double>
      perf_log_data_; // select, expand, gain, cost, value [cpu seconds]
  std::clock_t perf_cpu_timer_; // total time counter
  std::clock_t cpu_srv_timer_;  // To get CPU usage for service

  // params
  double p_replan_pos_threshold_; // m
  double p_replan_yaw_threshold_; // rad
  bool p_verbose_;
  bool p_visualize_; // Publish visualization of completed path, current gain,
                     // new candidates, ...
  bool p_publish_traversable_; // Same param as voxblox. Use for debugging, very
                               // detrimental to performance
  bool p_log_performance_;     // Whether to write a performance log file
  int p_max_new_segments_; // After this count is reached no more segments are
                           // expanded (0 to ignore)
  int p_min_new_segments_; // Until this count is reached the next segment is
                           // not executed (0 to ignore)
  int p_min_new_tries_;    // Until no. expansion calls the next segment is not
                           // executed (0 to ignore)
  int p_max_new_tries_; // After no. expansion calls the next segment is forced
                        // (0 to ignore)
  double p_min_new_value_; // Until this value is found in the tree, expansion
                           // continues (0 to ignore)
  int p_expand_batch_;     // run multiple segment expansions before rechecking
                           // conditions
  bool p_visualize_gain_; // true: add a colored sphere according to the gain to
                          // every segment
  // ideas: take images only on points,
  std::string logfile_;

  // methods
  void initializePlanning(); 

  virtual void loopIteration();

  void requestNextTrajectory();

  void expandTrajectories();

  void requestMovement(const TrajectorySegment &req);

  // visualization
  void publishTrajectoryVisualization(
      const std::vector<TrajectorySegment *> &trajectories);

  void publishCompletedTrajectoryVisualization(
      const TrajectorySegment &trajectories);

  void publishEvalVisualization(const TrajectorySegment &trajectory);

  // factory access
  void setupFromParamMap(Module::ParamMap *param_map) {}

  // helper functions
  bool checkMinNewValue(const std::unique_ptr<TrajectorySegment>
                            &segment); // check whether min value is found
};
} // namespace ros
} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_ROS_ROS_PLANNER_H_
