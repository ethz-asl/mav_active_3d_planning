#include "active_3d_planning_mav/module/trajectory_generator/feasible_rrt_star.h"

#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "active_3d_planning_core/data/system_constraints.h"

namespace active_3d_planning {

namespace trajectory_generator {

ModuleFactoryRegistry::Registration<FeasibleRRTStar>
    FeasibleRRTStar::registration("FeasibleRRTStar");

FeasibleRRTStar::FeasibleRRTStar(PlannerI& planner) : RRTStar(planner) {}

void FeasibleRRTStar::setupFromParamMap(Module::ParamMap* param_map) {
  // Setup parent and segment creator
  RRTStar::setupFromParamMap(param_map);
  setParam<bool>(param_map, "all_semgents_feasible", &p_all_semgents_feasible_,
                 false);
  segment_generator_.setConstraints(
      planner_.getSystemConstraints().v_max,
      planner_.getSystemConstraints().a_max,
      planner_.getSystemConstraints().yaw_rate_max,
      planner_.getSystemConstraints().yaw_accel_max, p_sampling_rate_);
}

bool FeasibleRRTStar::connectPoses(const EigenTrajectoryPoint& start,
                                   const EigenTrajectoryPoint& goal,
                                   EigenTrajectoryPointVector* result,
                                   bool check_collision) {
  if (check_collision) {
    // try creating a linear trajectory and check for collision
    Eigen::Vector3d direction = goal.position_W - start.position_W;
    int n_points =
        std::ceil(direction.norm() / planner_.getSystemConstraints().v_max *
                  p_sampling_rate_);
    for (int i = 0; i < n_points; ++i) {
      if (!checkTraversable(start.position_W +
                            static_cast<double>(i) /
                                static_cast<double>(n_points) * direction)) {
        return false;
      }
    }
  }
  if (p_all_semgents_feasible_) {
    return segment_generator_.createTrajectory(start, goal, result);
  } else {
    return segment_generator_.simulateTrajectory(start, goal, result);
  }
}

bool FeasibleRRTStar::extractTrajectoryToPublish(
    EigenTrajectoryPointVector* trajectory, const TrajectorySegment& segment) {
  if (p_all_semgents_feasible_) {
    // All segments are already feasible
    *trajectory = segment.trajectory;
    return true;
  } else {
    // Create a smooth semgent for execution
    if (segment_generator_.createTrajectory(segment.trajectory.front(),
                                            segment.trajectory.back(),
                                            trajectory)) {
      return true;
    } else {
      *trajectory = segment.trajectory;
      return false;
    }
  }
}

}  // namespace trajectory_generator
}  // namespace active_3d_planning
