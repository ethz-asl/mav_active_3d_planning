#define _USE_MATH_DEFINES

#include "active_3d_planning/module/trajectory_generator/feasible_rrt.h"

#include <cmath>
#include <memory>
#include <random>
#include <vector>

namespace active_3d_planning {
namespace trajectory_generator {

ModuleFactoryRegistry::Registration<FeasibleRRT>
    FeasibleRRT::registration("FeasibleRRT");

FeasibleRRT::FeasibleRRT(PlannerI &planner) : RRT(planner) {}

void FeasibleRRT::setupFromParamMap(Module::ParamMap *param_map) {
  // Setup parent and segment creator
  RRT::setupFromParamMap(param_map);
  setParam<bool>(param_map, "all_semgents_feasible", &p_all_semgents_feasible_,
                 false);
  segment_generator_.setConstraints(
      system_constraints_->v_max, system_constraints_->a_max,
      system_constraints_->yaw_rate_max, p_sampling_rate_);
}

bool FeasibleRRT::connectPoses(const EigenTrajectoryPoint &start,
                               const EigenTrajectoryPoint &goal,
                               EigenTrajectoryPointVector *result,
                               bool check_collision) {
  // try creating a linear trajectory and check for collision
  Eigen::Vector3d direction = goal.position_W - start.position_W;
  int n_points = std::ceil(direction.norm() /
                           (double)voxblox_.getEsdfMapPtr()->voxel_size());
  if(check_collision){
    for (int i = 0; i < n_points; ++i) {
      if (!checkTraversable(start.position_W +
                            (double)i / (double)n_points * direction)) {
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

bool FeasibleRRT::extractTrajectoryToPublish(
    EigenTrajectoryPointVector *trajectory, const TrajectorySegment &segment) {
  if (p_all_semgents_feasible_) {
    // All segments are already feasible
    *trajectory = segment.trajectory;
    return true;
  } else {
    // Create a smooth semgent for execution
    return segment_generator_.createTrajectory(
        segment.trajectory.front(), segment.trajectory.back(), trajectory);
  }
}

} // namespace trajectory_generator
} // namespace active_3d_planning