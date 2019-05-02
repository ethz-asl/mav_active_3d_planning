#define _USE_MATH_DEFINES

#include "mav_active_3d_planning/modules/trajectory_generators/feasible_rrt_star.h"

#include <mav_trajectory_generation/trajectory_sampling.h>

#include <vector>
#include <random>
#include <cmath>
#include <memory>

namespace mav_active_3d_planning {
    namespace trajectory_generators {

        ModuleFactory::Registration <FeasibleRRTStar> FeasibleRRTStar::registration("FeasibleRRTStar");

        void FeasibleRRTStar::setupFromParamMap(Module::ParamMap *param_map) {
            RRTStar::setupFromParamMap(param_map);
            typedef mav_trajectory_generation::InputConstraintType ICT;
            mav_trajectory_generation::InputConstraints input_constraints;
            input_constraints.addConstraint(ICT::kFMin, 0.5 * 9.81); // minimum acceleration in [m/s/s].
            input_constraints.addConstraint(ICT::kFMax, 1.5 * 9.81); // maximum acceleration in [m/s/s].
            input_constraints.addConstraint(ICT::kVMax, system_constraints_->v_max); // maximum velocity in [m/s].
            input_constraints.addConstraint(ICT::kOmegaXYMax, M_PI / 2.0); // maximum roll/pitch rates in [rad/s].
            input_constraints.addConstraint(ICT::kOmegaZMax, system_constraints_->yaw_rate_max); // max yaw rate [rad/s].
            input_constraints.addConstraint(ICT::kOmegaZDotMax, M_PI); // maximum yaw acceleration in [rad/s/s]..
            feasibility_check_ = mav_trajectory_generation::FeasibilityAnalytic(input_constraints);
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

        bool FeasibleRRTStar::connectPoses(const mav_msgs::EigenTrajectoryPoint &start,
                                           const mav_msgs::EigenTrajectoryPoint &goal,
                                           mav_msgs::EigenTrajectoryPointVector *result) {
            // try creating a linear trajectory and check for collision
            Eigen::Vector3d direction = goal.position_W - start.position_W;
            int n_points = std::ceil(direction.norm() / (double) voxblox_ptr_->getEsdfMapPtr()->voxel_size());
            for (int i = 0; i < n_points; ++i) {
                if (!checkTraversable(start.position_W + (double) i / (double) n_points * direction)) {
                    return false;
                }
            }
            const bool use_mav_trajectory_generation = true;
            if (use_mav_trajectory_generation){
                // create trajectory (4D)
                Eigen::Vector4d start4, goal4, vel4, acc4, zero4;
                start4 << start.position_W, start.getYaw();
                goal4 << goal.position_W, goal.getYaw();
                vel4 << start.velocity_W, start.getYawRate();
                acc4 << start.acceleration_W, start.getYawAcc();

                mav_trajectory_generation::Vertex::Vector vertices;
                mav_trajectory_generation::Vertex start_vert(4), goal_vert(4);
                start_vert.makeStartOrEnd(start4, mav_trajectory_generation::derivative_order::ACCELERATION);
                start_vert.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, vel4);
                start_vert.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, acc4);
                vertices.push_back(start_vert);
                goal_vert.makeStartOrEnd(goal4, mav_trajectory_generation::derivative_order::ACCELERATION);
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
                mav_msgs::EigenTrajectoryPoint::Vector states;
                if (!mav_trajectory_generation::sampleWholeTrajectory(trajectory, 1.0 / p_sampling_rate_, &states)) {
                    return false;
                }

                // Check collision
                if (!checkTrajectoryCollision(states)) {
                    return false;
                }

                // Build result
                *result = states;
                return true;
            } else {
                return FeasibleRRTStar::connectPosesFeasible(start, goal, result, system_constraints_->v_max,
                                                         system_constraints_->a_max,
                                                         system_constraints_->yaw_rate_max, p_sampling_rate_);
            }
        }

        void FeasibleRRTStar::optimizeVertices(mav_trajectory_generation::Vertex::Vector *vertices,
                                                       mav_trajectory_generation::Segment::Vector *segments,
                                                       mav_trajectory_generation::Trajectory *trajectory) {
            std::vector<double> segment_times = mav_trajectory_generation::estimateSegmentTimes(*vertices,
                                                                                                system_constraints_->v_max,
                                                                                                system_constraints_->a_max);
            mav_trajectory_generation::PolynomialOptimizationNonLinear<10> opt(4, parameters_);
            opt.setupFromVertices(*vertices, segment_times, mav_trajectory_generation::derivative_order::ACCELERATION);
            opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                                              system_constraints_->v_max);
            opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
                                              system_constraints_->a_max);
            opt.optimize();
            opt.getPolynomialOptimizationRef().getSegments(segments);
            opt.getTrajectory(trajectory);
        }

        bool FeasibleRRTStar::checkInputFeasible(const mav_trajectory_generation::Segment::Vector &segments) {
            for (int i = 0; i < segments.size(); ++i) {
                if (feasibility_check_.checkInputFeasibility(segments[i]) !=
                    mav_trajectory_generation::InputFeasibilityResult::kInputFeasible) {
                    return false;
                }
            }
            return true;
        }

        bool FeasibleRRTStar::checkTrajectoryCollision(const mav_msgs::EigenTrajectoryPoint::Vector &states) {
            for (int i = 0; i < states.size(); ++i) {
                if (!checkTraversable(states[i].position_W)) {
                    return false;
                }
            }
            return true;
        }

        bool FeasibleRRTStar::connectPosesFeasible(const mav_msgs::EigenTrajectoryPoint &start,
                                               const mav_msgs::EigenTrajectoryPoint &goal,
                                               mav_msgs::EigenTrajectoryPointVector *result, double v_max, double a_max,
                                               double yaw_rate_max, double sampling_rate) {
            // Build trajectory
            double current_yaw = start.getYaw();
            double goal_yaw = goal.getYaw();
            double v_curr = 0.0;
            double yaw_direction = defaults::angleDirection(current_yaw, goal_yaw);
            int64_t current_time = 0;
            Eigen::Vector3d current_pos = start.position_W;
            Eigen::Vector3d direction = goal.position_W - current_pos;
            direction /= direction.norm();
            result->clear();

            while (current_yaw != goal_yaw || current_pos != goal.position_W) {
                mav_msgs::EigenTrajectoryPoint trajectory_point;
                // Rotate until goal yaw reached
                if (current_yaw != goal_yaw) {
                    double delta_yaw = yaw_rate_max / sampling_rate;
                    if (defaults::angleDifference(current_yaw, goal_yaw) <= delta_yaw) {
                        current_yaw = goal_yaw;
                    } else {
                        current_yaw = defaults::angleScaled(current_yaw + yaw_direction * delta_yaw);
                    }
                }
                trajectory_point.setFromYaw(current_yaw);

                // Move until goal pos reached
                if (current_pos != goal.position_W) {
                    if ((current_pos - goal.position_W).norm() <= v_curr / sampling_rate) {
                        // goal reached
                        current_pos = goal.position_W;
                    } else if ((current_pos - goal.position_W).norm() > v_curr * v_curr / 2.0 / a_max) {
                        // Accelerate
                        v_curr = std::min(v_max, v_curr + a_max / sampling_rate);
                        current_pos += direction * v_curr / sampling_rate;
                    } else {
                        // Brake (minimum veloctiy to not get stuck in while loop)
                        v_curr = std::max(v_max * 0.05, v_curr - a_max / sampling_rate);
                        current_pos += direction * v_curr / sampling_rate;
                    }
                }
                trajectory_point.position_W = current_pos;

                // Keep track of time
                current_time += static_cast<int64_t>(1.0e9 / sampling_rate);
                trajectory_point.time_from_start_ns = current_time;
                result->push_back(trajectory_point);
            }
            return true;
        }

    } // namespace trajectory_generators
}  // namespace mav_active_3d_planning