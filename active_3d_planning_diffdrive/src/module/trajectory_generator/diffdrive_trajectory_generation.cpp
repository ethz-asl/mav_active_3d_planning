#include "active_3d_planning/module/trajectory_generator/diffdrive_trajectory_generation.h"

#include <cmath>
#include <random>

namespace active_3d_planning {
namespace trajectory_generator {

ModuleFactoryRegistry::Registration<DiffDriveTrajectoryGeneration>
    DiffDriveTrajectoryGeneration::registration(
        "DiffDriveTrajectoryGeneration");

DiffDriveTrajectoryGeneration::DiffDriveTrajectoryGeneration(PlannerI &planner)
    : TrajectoryGenerator(planner) {}

void DiffDriveTrajectoryGeneration::setupFromParamMap(
    Module::ParamMap *param_map) {
  TrajectoryGenerator::setupFromParamMap(param_map);
  setParam<double>(param_map, "distance_max", &p_distance_max_, 3.0);
  setParam<double>(param_map, "distance_min", &p_distance_min_, 0.1);
  setParam<double>(param_map, "sampling_rate", &p_sampling_rate_, 0.5);
  setParam<int>(param_map, "n_segments", &p_n_segments_, 5);
  setParam<int>(param_map, "max_tries", &p_max_tries_, 100);
  setParam<double>(param_map, "max_dyaw", &max_dyaw_, 1);
  setParam<double>(param_map, "max_acc", &max_acc_, 1);
  setParam<double>(param_map, "max_vel", &max_vel_, 4);
  setParam<double>(param_map, "max_time", &max_time_, 5);
}

bool DiffDriveTrajectoryGeneration::checkParamsValid(
    std::string *error_message) {
  if (p_distance_max_ <= 0.0) {
    *error_message = "distance_max expected > 0.0";
    return false;
  } else if (p_distance_max_ < p_distance_min_) {
    *error_message = "distance_max needs to be larger than distance_min";
    return false;
  } else if (p_n_segments_ < 1) {
    *error_message = "n_segments expected > 0";
    return false;
  } else if (p_max_tries_ < 1) {
    *error_message = "max_tries expected > 0";
    return false;
  }
  return true;
}

bool DiffDriveTrajectoryGeneration::expandSegment(
    TrajectorySegment *target, std::vector<TrajectorySegment *> *new_segments) {
  // Create and add new adjacent trajectories to target segment
  target->tg_visited = true;
  int valid_segments = 0;
  Eigen::Vector3d start_pos = target->trajectory.back().position_W;
  double start_vel_B_x = target->trajectory.back().velocity_W.squaredNorm();
  double start_yaw_W =
      std::acos(target->trajectory.back().orientation_W_B.w()) *
      2; // because we know only yaw can be non zero
  int counter = 0;

  while (valid_segments < p_n_segments_ && counter < p_max_tries_) {
    counter++;
    // for now super simple, sample final velocity + yaw speed (need to later
    // smooth somehow and stuff) yaw jumps atm
    double dyaw = double(rand()) / RAND_MAX * 2 * max_dyaw_ - max_dyaw_;
    double target_vel_B_x = double(rand()) / RAND_MAX * 2 * max_vel_ - max_vel_;
    double distance =
        double(rand()) / RAND_MAX * (p_distance_max_ - p_distance_min_) +
        p_distance_min_;
    auto time = std::abs(distance / ((start_vel_B_x + target_vel_B_x) / 2));
    auto acc = (target_vel_B_x - start_vel_B_x) / time;
    if (std::abs(acc) > max_acc_ || time > max_time_) {
      continue;
    }
    // std::cout << "new segment"<<std::endl;
    EigenTrajectoryPoint::Vector states;
    auto currpos =
        start_pos; // getting position via super simple integration (1st order)
    for (int i = 0; i <= time * p_sampling_rate_; ++i) {
      auto time_from_start = double(i) / p_sampling_rate_;

      auto curryaw = start_yaw_W + dyaw * time_from_start;
      auto currorientation_W_B = Eigen::Quaterniond(std::cos(curryaw / 2), 0, 0,
                                                    std::sin(curryaw / 2));
      auto currvel_W =
          currorientation_W_B *
          Eigen::Vector3d(start_vel_B_x + acc * time_from_start, 0, 0);
      currpos += 1.0 / p_sampling_rate_ * currvel_W;
      EigenTrajectoryPoint trajectory_point;
      trajectory_point.position_W = currpos;
      trajectory_point.setFromYaw(defaults::angleScaled(curryaw));
      // std::cout << i << " gives " << static_cast<int64_t>(time_from_start
      // * 1.0e9) << std::endl; seems ok
      trajectory_point.time_from_start_ns =
          static_cast<int64_t>(time_from_start * 1.0e9);
      states.push_back(trajectory_point);
    }

    // Check collision
    if (!checkTrajectoryCollision(states)) {
      continue;
    }

    // Build result
    TrajectorySegment *new_segment = target->spawnChild();
    new_segment->trajectory = states;
    new_segments->push_back(new_segment);
    valid_segments++;
  }
  // Feasible solution found?
  return (valid_segments > 0);
}


bool DiffDriveTrajectoryGeneration::checkTrajectoryCollision(
    const EigenTrajectoryPoint::Vector &states) {
  for (int i = 0; i < states.size(); ++i) {
    if (!checkTraversable(states[i].position_W)) {
      return false;
    }
  }
  return true;
}

} // namespace trajectory_generator
} // namespace active_3d_planning