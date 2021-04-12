#include "active_3d_planning_mav/module/trajectory_generator/random_mav_trajectory.h"

#include <random>
#include <string>
#include <vector>

#include <mav_trajectory_generation/trajectory_sampling.h>

#include "active_3d_planning_core/data/system_constraints.h"
#include "active_3d_planning_mav/tools/tools.h"

namespace active_3d_planning {
namespace trajectory_generator {

ModuleFactoryRegistry::Registration<RandomMavTrajectory>
    RandomMavTrajectory::registration("RandomMavTrajectory");

RandomMavTrajectory::RandomMavTrajectory(PlannerI& planner)
    : TrajectoryGenerator(planner) {}

void RandomMavTrajectory::setupFromParamMap(Module::ParamMap* param_map) {
  TrajectoryGenerator::setupFromParamMap(param_map);
  setParam<double>(param_map, "distance_max", &p_distance_max_, 3.0);
  setParam<double>(param_map, "distance_min", &p_distance_min_, 0.5);
  setParam<double>(param_map, "sampling_rate", &p_sampling_rate_, 0.5);
  setParam<int>(param_map, "n_segments", &p_n_segments_, 5);
  setParam<int>(param_map, "max_tries", &p_max_tries_, 100);
  initializeConstraints();
}

bool RandomMavTrajectory::checkParamsValid(std::string* error_message) {
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

void RandomMavTrajectory::initializeConstraints() {
  // Create input constraints.for feasibility check
  // todo: ev set these from params / system_constraints_mav
  typedef mav_trajectory_generation::InputConstraintType ICT;
  mav_trajectory_generation::InputConstraints input_constraints;
  input_constraints.addConstraint(
      ICT::kFMin, 0.5 * 9.81);  // minimum acceleration in [m/s/s].
  input_constraints.addConstraint(
      ICT::kFMax, 1.5 * 9.81);  // maximum acceleration in [m/s/s].
  input_constraints.addConstraint(
      ICT::kVMax,
      planner_.getSystemConstraints().v_max);  // maximum velocity in [m/s].
  input_constraints.addConstraint(
      ICT::kOmegaXYMax, M_PI / 2.0);  // maximum roll/pitch rates in [rad/s].
  input_constraints.addConstraint(
      ICT::kOmegaZMax,
      planner_.getSystemConstraints().yaw_rate_max);  // max yaw rate [rad/s].
  input_constraints.addConstraint(
      ICT::kOmegaZDotMax,
      planner_.getSystemConstraints()
          .yaw_accel_max);  // maximum yaw acceleration in [rad/s/s]..
  feasibility_check_ =
      mav_trajectory_generation::FeasibilityAnalytic(input_constraints);
  feasibility_check_.settings_.setMinSectionTimeS(0.01);

  // Create nonlinear optimizer (Commented out since defaults should do)
  //        parameters_.max_iterations = 1000;
  //        parameters_.f_rel = 0.05;
  //        parameters_.x_rel = 0.1;
  //        parameters_.time_penalty = 500.0;
  //        parameters_.initial_stepsize_rel = 0.1;
  //        parameters_.inequality_constraint_tolerance = 0.1;

  parameters_ = mav_trajectory_generation::NonlinearOptimizationParameters();
}

bool RandomMavTrajectory::expandSegment(
    TrajectorySegment* target, std::vector<TrajectorySegment*>* new_segments) {
  // Create and add new adjacent trajectories to target segment
  target->tg_visited = true;
  int valid_segments = 0;
  Eigen::Vector3d start_pos = target->trajectory.back().position_W;
  int counter = 0;

  while (valid_segments < p_n_segments_ && counter < p_max_tries_) {
    counter++;
    // new random target
    double yaw;
    Eigen::Vector3d goal_pos;
    sampleGoalPose(&yaw, &goal_pos, start_pos);

    // create trajectory (4D)
    Eigen::Vector4d start4, goal4, vel4, acc4;
    start4 << start_pos, target->trajectory.back().getYaw();
    goal4 << goal_pos, yaw;
    vel4 << target->trajectory.back().velocity_W,
        target->trajectory.back().getYawRate();
    acc4 << target->trajectory.back().acceleration_W,
        target->trajectory.back().getYawAcc();

    mav_trajectory_generation::Vertex::Vector vertices;
    mav_trajectory_generation::Vertex start(4), goal(4);
    start.makeStartOrEnd(
        start4, mav_trajectory_generation::derivative_order::ACCELERATION);
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        vel4);
    start.addConstraint(
        mav_trajectory_generation::derivative_order::ACCELERATION, acc4);
    vertices.push_back(start);
    goal.makeStartOrEnd(
        goal4, mav_trajectory_generation::derivative_order::ACCELERATION);

    // test: add some goal velocity
    Eigen::Vector4d constraint4;
    constraint4 << 0.5 * (goal_pos - start_pos).normalized(), 0;
    goal.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                       constraint4);
    vertices.push_back(goal);

    // Optimize
    mav_trajectory_generation::Segment::Vector segments;
    mav_trajectory_generation::Trajectory trajectory;
    optimizeVertices(&vertices, &segments, &trajectory);

    // check input feasibility
    if (!checkInputFeasible(segments)) {
      continue;
    }

    // convert to EigenTrajectory
    mav_msgs::EigenTrajectoryPoint::Vector tmp_states;
    if (!mav_trajectory_generation::sampleWholeTrajectory(
            trajectory, 1.0 / p_sampling_rate_, &tmp_states)) {
      continue;
    }

    EigenTrajectoryPointVector states = mavMsgToPlanningMsg(tmp_states);

    // Check collision
    if (!checkTrajectoryCollision(states)) {
      continue;
    }

    // Build result
    TrajectorySegment* new_segment = target->spawnChild();
    new_segment->trajectory = states;
    new_segments->push_back(new_segment);
    valid_segments++;
  }
  // Feasible solution found?
  return (valid_segments > 0);
}

void RandomMavTrajectory::sampleGoalPose(double* yaw, Eigen::Vector3d* goal_pos,
                                         const Eigen::Vector3d& start_pos) {
  *yaw = static_cast<double>(rand()) * 2.0 * M_PI / RAND_MAX - M_PI;
  double phi = static_cast<double>(rand()) * 2.0 * M_PI / RAND_MAX;
  double theta = static_cast<double>(rand()) * M_PI / RAND_MAX;
  double range = p_distance_min_ + static_cast<double>(rand()) / RAND_MAX *
                                       (p_distance_max_ - p_distance_min_);
  *goal_pos =
      start_pos + range * Eigen::Vector3d(sin(theta) * cos(phi),
                                          sin(theta) * sin(phi), cos(theta));
}

void RandomMavTrajectory::optimizeVertices(
    mav_trajectory_generation::Vertex::Vector* vertices,
    mav_trajectory_generation::Segment::Vector* segments,
    mav_trajectory_generation::Trajectory* trajectory) {
  std::vector<double> segment_times =
      mav_trajectory_generation::estimateSegmentTimes(
          *vertices, planner_.getSystemConstraints().v_max,
          planner_.getSystemConstraints().a_max);
  mav_trajectory_generation::PolynomialOptimizationNonLinear<10> opt(
      4, parameters_);
  opt.setupFromVertices(
      *vertices, segment_times,
      mav_trajectory_generation::derivative_order::ACCELERATION);
  opt.addMaximumMagnitudeConstraint(
      mav_trajectory_generation::derivative_order::VELOCITY,
      planner_.getSystemConstraints().v_max);
  opt.addMaximumMagnitudeConstraint(
      mav_trajectory_generation::derivative_order::ACCELERATION,
      planner_.getSystemConstraints().a_max);
  opt.optimize();
  opt.getPolynomialOptimizationRef().getSegments(segments);
  opt.getTrajectory(trajectory);
}

bool RandomMavTrajectory::checkInputFeasible(
    const mav_trajectory_generation::Segment::Vector& segments) {
  for (int i = 0; i < segments.size(); ++i) {
    if (feasibility_check_.checkInputFeasibility(segments[i]) !=
        mav_trajectory_generation::InputFeasibilityResult::kInputFeasible) {
      return false;
    }
  }
  return true;
}

bool RandomMavTrajectory::checkTrajectoryCollision(
    const EigenTrajectoryPoint::Vector& states) {
  for (int i = 0; i < states.size(); ++i) {
    if (!checkTraversable(states[i].position_W)) {
      return false;
    }
  }
  return true;
}

}  // namespace trajectory_generator
}  // namespace active_3d_planning
