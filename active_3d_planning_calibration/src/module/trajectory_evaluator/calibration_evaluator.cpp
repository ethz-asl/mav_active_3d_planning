#include "active_3d_planning/module/trajectory_evaluator/calibration_evaluator.h"

#include <algorithm>
#include <limits>

#include <geometry_msgs/PoseStamped.h>

#include <active_calibration_comm/EvaluateTrajectoryService.h>

namespace active_3d_planning {
namespace trajectory_evaluator {

// Factory Registration
ModuleFactoryRegistry::Registration<CalibrationEvaluator>
    CalibrationEvaluator::registration("CalibrationEvaluator");

CalibrationEvaluator::CalibrationEvaluator(PlannerI &planner)
    : SimulatedSensorEvaluator(planner) {}

void CalibrationEvaluator::setupFromParamMap(Module::ParamMap *param_map) {
  // setup parent
  SimulatedSensorEvaluator::setupFromParamMap(param_map);
  setParam<std::string>(param_map, "evaluation_service_name",
                        &evaluation_service_name, "");
  evaluation_client =
      node_handle
          .serviceClient<active_calibration_comm::EvaluateTrajectoryService>(
              evaluation_service_name);
  pathCompletionListener = node_handle.subscribe(
      "target_reached", 1, &CalibrationEvaluator::resetMap, this);
}

bool CalibrationEvaluator::computeGainFromVisibleVoxels(
    TrajectorySegment *traj_in) {
  if (!traj_in->info) {
    traj_in->gain = std::numeric_limits<double>::min();
    return false;
  }

  if (trajIdMap.find(traj_in) != trajIdMap.end()) {
    ROS_ERROR(
        "Trajectory already found in map, this shouldnt happen, skipping!");
    traj_in->gain = std::numeric_limits<double>::min();
    return false;
  }
  if (traj_in->parent && trajIdMap.find(traj_in->parent) == trajIdMap.end()) {
    ROS_WARN(
        "Attempting to evaluate child before parent, evaluating parent first!");
    computeGainFromVisibleVoxels(traj_in->parent);
  }
  trajIdMap.emplace(traj_in, idCounter++);

  // call service
  std::vector<geometry_msgs::PoseStamped> poses_stamped;
  for (auto &&eigen_trajectory_point_vector : traj_in->trajectory) {
    geometry_msgs::PoseStamped p;
    
    p.header.stamp.fromNSec(eigen_trajectory_point_vector.time_from_start_ns);
    
    p.pose.position.x = eigen_trajectory_point_vector.position_W.x();
    p.pose.position.y = eigen_trajectory_point_vector.position_W.y();
    p.pose.position.z = eigen_trajectory_point_vector.position_W.z();

    p.pose.orientation.x = eigen_trajectory_point_vector.orientation_W_B.x();
    p.pose.orientation.y = eigen_trajectory_point_vector.orientation_W_B.y();
    p.pose.orientation.z = eigen_trajectory_point_vector.orientation_W_B.z();
    p.pose.orientation.w = eigen_trajectory_point_vector.orientation_W_B.w();

    poses_stamped.push_back(p);
  }
  active_calibration_comm::EvaluateTrajectoryService srv;
  srv.request.posesStamped = poses_stamped;
  if (traj_in->parent) {
    srv.request.parent = trajIdMap[traj_in->parent];
  } else {
    srv.request.parent = -1;
  }
  srv.request.id = trajIdMap[traj_in];
  if (evaluation_client.call(srv)) {
    traj_in->gain = srv.response.quality;
    return true;
  } else {
    traj_in->gain = std::numeric_limits<double>::min();
    return false;
  }
}

void CalibrationEvaluator::resetMap(const std_msgs::Empty & /* msg */) {
  idCounter = 0;
  trajIdMap.clear();
}
} // namespace trajectory_evaluator
} // namespace active_3d_planning
