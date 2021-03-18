#define _USE_MATH_DEFINES

#include "active_3d_planning_core/module/trajectory_generator/random_linear.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "active_3d_planning_core/data/system_constraints.h"
#include "active_3d_planning_core/data/trajectory.h"
#include "active_3d_planning_core/tools/defaults.h"

namespace active_3d_planning {
namespace trajectory_generator {

ModuleFactoryRegistry::Registration<RandomLinear> RandomLinear::registration(
    "RandomLinear");

RandomLinear::RandomLinear(PlannerI& planner) : TrajectoryGenerator(planner) {}

void RandomLinear::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<double>(param_map, "min_distance", &p_min_distance_, 1.0);
  setParam<double>(param_map, "max_distance", &p_max_distance_, 1.0);
  setParam<double>(param_map, "sampling_rate", &p_sampling_rate_, 20.0);
  setParam<int>(param_map, "n_segments", &p_n_segments_, 5);
  setParam<int>(param_map, "max_tries", &p_max_tries_, 1000);
  setParam<bool>(param_map, "planar", &p_planar_, true);
  setParam<bool>(param_map, "sample_yaw", &p_sample_yaw_, false);

  // setup parent
  TrajectoryGenerator::setupFromParamMap(param_map);
}

bool RandomLinear::checkParamsValid(std::string* error_message) {
  if (p_max_distance_ <= 0.0) {
    *error_message = "max_distance expected > 0.0";
    return false;
  } else if (p_max_distance_ < p_min_distance_) {
    *error_message = "max_distance needs to be larger than min_distance";
    return false;
  } else if (p_n_segments_ < 1) {
    *error_message = "n_segments expected > 0";
    return false;
  } else if (p_max_tries_ < 1) {
    *error_message = "max_tries expected > 0";
    return false;
  }
  return TrajectoryGenerator::checkParamsValid(error_message);
}

bool RandomLinear::expandSegment(
    TrajectorySegment* target, std::vector<TrajectorySegment*>* new_segments) {
  // Create and add new adjacent trajectories to target segment
  target->tg_visited = true;
  int valid_segments = 0;
  int counter = 0;
  TrajectorySegment* new_segment = target->spawnChild();
  Eigen::Vector3d start_pos = target->trajectory.back().position_W;

  Eigen::Vector3d direction;
  double yaw;
  double decelleration_distance;

  while (valid_segments < p_n_segments_ && counter < p_max_tries_) {
    counter++;

    // new random target selection
    sampleTarget(&direction, &yaw, &decelleration_distance);

    // Try building the trajectory and check if it's collision free
    if (buildTrajectory(start_pos, direction, yaw, decelleration_distance,
                        new_segment)) {
      valid_segments++;
      new_segments->push_back(new_segment);
      new_segment = target->spawnChild();
    }
  }
  target->children.pop_back();

  // Feasible solution found?
  return (valid_segments > 0);
}

void RandomLinear::sampleTarget(Eigen::Vector3d* direction, double* yaw,
                                double* decelleration_distance) {
  // new random target direction
  *yaw = static_cast<double>(rand()) * 2.0 * M_PI / RAND_MAX;
  double theta = static_cast<double>(rand()) * M_PI / RAND_MAX;
  double distance = p_min_distance_ + static_cast<double>(rand()) / RAND_MAX *
                                          (p_max_distance_ - p_min_distance_);
  *decelleration_distance =
      distance - std::min(std::pow(planner_.getSystemConstraints().v_max, 2.0) /
                              planner_.getSystemConstraints().a_max,
                          distance) /
                     2;
  if (p_planar_) {
    theta = 0.5 * M_PI;
  }
  *direction = Eigen::Vector3d(sin(theta) * cos(*yaw), sin(theta) * sin(*yaw),
                               cos(theta));
  if (p_sample_yaw_) {
    *yaw =
        static_cast<double>(rand()) * 2.0 * M_PI / RAND_MAX;  // new orientation
  }
}

bool RandomLinear::buildTrajectory(const Eigen::Vector3d& start_pos,
                                   const Eigen::Vector3d& direction, double yaw,
                                   double decelleration_distance,
                                   TrajectorySegment* new_segment) {
  // Simulate trajectory (accelerate and deccelerate to 0 velocity at goal
  // points)
  double x_curr = 0.0, v_curr = 0.0, t_curr = 0.0;

  while (v_curr >= 0.0) {
    if (x_curr < decelleration_distance) {
      v_curr = std::min(
          v_curr + planner_.getSystemConstraints().a_max / p_sampling_rate_,
          planner_.getSystemConstraints().v_max);
    } else {
      v_curr -= planner_.getSystemConstraints().a_max / p_sampling_rate_;
    }
    t_curr += 1.0 / p_sampling_rate_;
    x_curr += v_curr / p_sampling_rate_;
    Eigen::Vector3d current_pos = start_pos + x_curr * direction;

    // check collision
    if (!checkTraversable(current_pos)) {
      new_segment->trajectory.clear();
      return false;
    }

    // append to result
    EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W = current_pos;
    trajectory_point.setFromYaw(defaults::angleScaled(yaw));
    trajectory_point.time_from_start_ns = static_cast<int64_t>(t_curr * 1.0e9);
    new_segment->trajectory.push_back(trajectory_point);
  }
  return true;
}

}  // namespace trajectory_generator
}  // namespace active_3d_planning
