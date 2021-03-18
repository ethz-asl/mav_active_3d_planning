#define _USE_MATH_DEFINES

#include "active_3d_planning_mav/tools/linear_mav_trajectory_generator.h"

#include <algorithm>
#include <vector>

#include <mav_trajectory_generation/trajectory_sampling.h>

#include "active_3d_planning_core/tools/defaults.h"
#include "active_3d_planning_mav/tools/tools.h"

namespace active_3d_planning {

void LinearMavTrajectoryGenerator::setConstraints(double v_max, double a_max,
                                                  double yaw_rate_max,
                                                  double yaw_accel_max,
                                                  double sampling_rate) {
  // Other constraints are currently not exposed (could also be)
  typedef mav_trajectory_generation::InputConstraintType ICT;
  mav_trajectory_generation::InputConstraints input_constraints;
  input_constraints.addConstraint(
      ICT::kFMin, 0.5 * 9.81);  // minimum acceleration in [m/s/s].
  input_constraints.addConstraint(
      ICT::kFMax, 1.5 * 9.81);  // maximum acceleration in [m/s/s].
  input_constraints.addConstraint(ICT::kVMax,
                                  v_max);  // maximum velocity in [m/s].
  input_constraints.addConstraint(
      ICT::kOmegaXYMax, M_PI / 2.0);  // maximum roll/pitch rates in [rad/s].
  input_constraints.addConstraint(ICT::kOmegaZMax,
                                  yaw_rate_max);  // max yaw rate [rad/s].
  input_constraints.addConstraint(
      ICT::kOmegaZDotMax,
      yaw_accel_max);  // maximum yaw acceleration in [rad/s/s]..
  feasibility_check_ =
      mav_trajectory_generation::FeasibilityAnalytic(input_constraints);
  feasibility_check_.settings_.setMinSectionTimeS(0.01);

  // Create nonlinear optimizer
  parameters_ = mav_trajectory_generation::NonlinearOptimizationParameters();
  //        (Commented out since defaults should do)
  //        parameters_.max_iterations = 1000;
  //        parameters_.f_rel = 0.05;
  //        parameters_.x_rel = 0.1;
  //        parameters_.time_penalty = 500.0;
  //        parameters_.initial_stepsize_rel = 0.1;
  //        parameters_.inequality_constraint_tolerance = 0.1;

  // cache constraints
  v_max_ = v_max;
  a_max_ = a_max;
  yaw_rate_max_ = yaw_rate_max;
  yaw_accel_max_ = yaw_accel_max;
  sampling_rate_ = sampling_rate;
}

bool LinearMavTrajectoryGenerator::createTrajectory(
    const EigenTrajectoryPoint& start, const EigenTrajectoryPoint& goal,
    EigenTrajectoryPointVector* result) {
  // create trajectory (4D)
  Eigen::Vector4d start4, goal4, vel4, acc4;
  start4 << start.position_W, start.getYaw();
  goal4 << goal.position_W, goal.getYaw();
  vel4 << start.velocity_W, start.getYawRate();
  acc4 << start.acceleration_W, start.getYawAcc();
  for (int i = 0; i < 4; ++i) {
    if (start4[i] == goal4[i]) {
      goal4[i] =
          goal4[i] +
          1e-3;  // If a dimension has same start and stop value nlopt dies ?!
    }
  }

  mav_trajectory_generation::Vertex::Vector vertices;
  mav_trajectory_generation::Vertex start_vert(4), goal_vert(4);
  start_vert.makeStartOrEnd(
      start4, mav_trajectory_generation::derivative_order::ACCELERATION);
  start_vert.addConstraint(
      mav_trajectory_generation::derivative_order::VELOCITY, vel4);
  start_vert.addConstraint(
      mav_trajectory_generation::derivative_order::ACCELERATION, acc4);
  vertices.push_back(start_vert);
  goal_vert.makeStartOrEnd(
      goal4, mav_trajectory_generation::derivative_order::ACCELERATION);
  vertices.push_back(goal_vert);

  // Optimize
  mav_trajectory_generation::Segment::Vector segments;
  mav_trajectory_generation::Trajectory trajectory;
  optimizeVertices(&vertices, &segments, &trajectory);

  // check input feasibility
  if (!checkInputFeasible(segments)) {
    return false;
  }

  // convert to EigenTrajectory
  mav_msgs::EigenTrajectoryPoint::Vector tmp_states;
  if (!mav_trajectory_generation::sampleWholeTrajectory(
          trajectory, 1.0 / sampling_rate_, &tmp_states)) {
    return false;
  }
  EigenTrajectoryPoint::Vector states = mavMsgToPlanningMsg(tmp_states);

  // Build result
  *result = states;
  return true;
}

void LinearMavTrajectoryGenerator::optimizeVertices(
    mav_trajectory_generation::Vertex::Vector* vertices,
    mav_trajectory_generation::Segment::Vector* segments,
    mav_trajectory_generation::Trajectory* trajectory) {
  // Taken from mav_trajectory_generation routine
  std::vector<double> segment_times =
      mav_trajectory_generation::estimateSegmentTimes(*vertices, v_max_,
                                                      a_max_);
  mav_trajectory_generation::PolynomialOptimizationNonLinear<10> opt(
      4, parameters_);
  opt.setupFromVertices(
      *vertices, segment_times,
      mav_trajectory_generation::derivative_order::ACCELERATION);
  opt.addMaximumMagnitudeConstraint(
      mav_trajectory_generation::derivative_order::VELOCITY, v_max_);
  opt.addMaximumMagnitudeConstraint(
      mav_trajectory_generation::derivative_order::ACCELERATION, a_max_);
  opt.optimize();
  opt.getPolynomialOptimizationRef().getSegments(segments);
  opt.getTrajectory(trajectory);
}

bool LinearMavTrajectoryGenerator::checkInputFeasible(
    const mav_trajectory_generation::Segment::Vector& segments) {
  // input feasibility check
  for (int i = 0; i < segments.size(); ++i) {
    if (feasibility_check_.checkInputFeasibility(segments[i]) !=
        mav_trajectory_generation::InputFeasibilityResult::kInputFeasible) {
      return false;
    }
  }
  return true;
}

bool LinearMavTrajectoryGenerator::simulateTrajectory(
    const EigenTrajectoryPoint& start, const EigenTrajectoryPoint& goal,
    EigenTrajectoryPointVector* result) {
  // Build trajectory
  double current_yaw = defaults::angleScaled(start.getYaw());
  double goal_yaw = defaults::angleScaled(goal.getYaw());
  double v_curr = 0.0;
  double yaw_rate_curr = 0.0;
  double yaw_direction = defaults::angleDirection(current_yaw, goal_yaw);
  int64_t current_time = 0;
  Eigen::Vector3d current_pos = start.position_W;
  Eigen::Vector3d direction = goal.position_W - current_pos;
  direction /= direction.norm();
  result->clear();
  {
    EigenTrajectoryPoint trajectory_point;
    trajectory_point.setFromYaw(current_yaw);
    trajectory_point.position_W = current_pos;
    trajectory_point.time_from_start_ns = current_time;
    result->push_back(trajectory_point);
  }

  while (current_yaw != goal_yaw || current_pos != goal.position_W) {
    EigenTrajectoryPoint trajectory_point;
    // Rotate until goal yaw reached
    if (current_yaw != goal_yaw) {
      if (defaults::angleDifference(current_yaw, goal_yaw) <=
          yaw_rate_curr / sampling_rate_) {
        // goal reached
        current_yaw = goal_yaw;
      } else if (defaults::angleDifference(current_yaw, goal_yaw) >
                 yaw_rate_curr * yaw_rate_curr / 2.0 / yaw_accel_max_) {
        // Accelerate
        yaw_rate_curr = std::min(
            yaw_rate_max_, yaw_rate_curr + yaw_accel_max_ / sampling_rate_);
        current_yaw = defaults::angleScaled(
            current_yaw + yaw_direction * yaw_rate_curr / sampling_rate_);
      } else {
        // Brake (minimum veloctiy to not get stuck in while loop)
        yaw_rate_curr =
            std::max(yaw_rate_max_ * 0.05,
                     yaw_rate_curr - yaw_accel_max_ / sampling_rate_);
        current_yaw = defaults::angleScaled(
            current_yaw + yaw_direction * yaw_rate_curr / sampling_rate_);
      }
    }
    trajectory_point.setFromYaw(current_yaw);

    // Move until goal pos reached
    if (current_pos != goal.position_W) {
      if ((current_pos - goal.position_W).norm() <= v_curr / sampling_rate_) {
        // goal reached
        current_pos = goal.position_W;
      } else if ((current_pos - goal.position_W).norm() >
                 v_curr * v_curr / 2.0 / a_max_) {
        // Accelerate
        v_curr = std::min(v_max_, v_curr + a_max_ / sampling_rate_);
        current_pos += direction * v_curr / sampling_rate_;
      } else {
        // Brake (minimum veloctiy to not get stuck in while loop)
        v_curr = std::max(v_max_ * 0.05, v_curr - a_max_ / sampling_rate_);
        current_pos += direction * v_curr / sampling_rate_;
      }
    }
    trajectory_point.position_W = current_pos;

    // Keep track of time
    current_time += static_cast<int64_t>(1.0e9 / sampling_rate_);
    trajectory_point.time_from_start_ns = current_time;
    result->push_back(trajectory_point);
  }
  return true;
}

}  // namespace active_3d_planning
