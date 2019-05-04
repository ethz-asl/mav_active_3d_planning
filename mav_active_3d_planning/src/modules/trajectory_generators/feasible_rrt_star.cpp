#define _USE_MATH_DEFINES

#include "mav_active_3d_planning/modules/trajectory_generators/feasible_rrt_star.h"

#include <mav_trajectory_generation/trajectory_sampling.h>

#include <vector>
#include <random>
#include <cmath>
#include <memory>

namespace mav_active_3d_planning {

    void LinearMavTrajectoryGeneration::setConstraints(double v_max, double a_max, double yaw_rate_max,
                                                       double sampling_rate) {
        // Other constraints are currently not exposed (could also be)
        typedef mav_trajectory_generation::InputConstraintType ICT;
        mav_trajectory_generation::InputConstraints input_constraints;
        input_constraints.addConstraint(ICT::kFMin, 0.5 * 9.81); // minimum acceleration in [m/s/s].
        input_constraints.addConstraint(ICT::kFMax, 1.5 * 9.81); // maximum acceleration in [m/s/s].
        input_constraints.addConstraint(ICT::kVMax, v_max); // maximum velocity in [m/s].
        input_constraints.addConstraint(ICT::kOmegaXYMax, M_PI / 2.0); // maximum roll/pitch rates in [rad/s].
        input_constraints.addConstraint(ICT::kOmegaZMax, yaw_rate_max); // max yaw rate [rad/s].
        input_constraints.addConstraint(ICT::kOmegaZDotMax, M_PI); // maximum yaw acceleration in [rad/s/s]..
        feasibility_check_ = mav_trajectory_generation::FeasibilityAnalytic(input_constraints);
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
        sampling_rate_ = sampling_rate;
    }

    bool LinearMavTrajectoryGeneration::createTrajectory(const mav_msgs::EigenTrajectoryPoint &start,
                                                         const mav_msgs::EigenTrajectoryPoint &goal,
                                                         mav_msgs::EigenTrajectoryPointVector *result) {
        // create trajectory (4D)
        Eigen::Vector4d start4, goal4, vel4, acc4;
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
        if (!mav_trajectory_generation::sampleWholeTrajectory(trajectory, 1.0 / sampling_rate_, &states)) {
            return false;
        }

        // Build result
        *result = states;
        return true;
    }

    void LinearMavTrajectoryGeneration::optimizeVertices(mav_trajectory_generation::Vertex::Vector *vertices,
                                                         mav_trajectory_generation::Segment::Vector *segments,
                                                         mav_trajectory_generation::Trajectory *trajectory) {
        // Taken from mav_trajectory_generation routine
        std::vector<double> segment_times = mav_trajectory_generation::estimateSegmentTimes(*vertices, v_max_, a_max_);
        mav_trajectory_generation::PolynomialOptimizationNonLinear<10> opt(4, parameters_);
        opt.setupFromVertices(*vertices, segment_times, mav_trajectory_generation::derivative_order::ACCELERATION);
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max_);
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max_);
        opt.optimize();
        opt.getPolynomialOptimizationRef().getSegments(segments);
        opt.getTrajectory(trajectory);
    }

    bool LinearMavTrajectoryGeneration::checkInputFeasible(const mav_trajectory_generation::Segment::Vector &segments) {
        // input feasibility check
        for (int i = 0; i < segments.size(); ++i) {
            if (feasibility_check_.checkInputFeasibility(segments[i]) !=
                mav_trajectory_generation::InputFeasibilityResult::kInputFeasible) {
                return false;
            }
        }
        return true;
    }


    namespace trajectory_generators {

        ModuleFactory::Registration <FeasibleRRT> FeasibleRRT::registration("FeasibleRRT");

        void FeasibleRRT::setupFromParamMap(Module::ParamMap *param_map) {
            // Setup parent and segment creator
            RRT::setupFromParamMap(param_map);
            segment_generator_.setConstraints(system_constraints_->v_max, system_constraints_->a_max,
                                              system_constraints_->yaw_rate_max, p_sampling_rate_);
        }

        bool FeasibleRRT::connectPoses(const mav_msgs::EigenTrajectoryPoint &start,
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
            return segment_generator_.createTrajectory(start, goal, result);
        }

        ModuleFactory::Registration <FeasibleRRTStar> FeasibleRRTStar::registration("FeasibleRRTStar");

        void FeasibleRRTStar::setupFromParamMap(Module::ParamMap *param_map) {
            // Setup parent and segment creator
            RRTStar::setupFromParamMap(param_map);
            segment_generator_.setConstraints(system_constraints_->v_max, system_constraints_->a_max,
                                              system_constraints_->yaw_rate_max, p_sampling_rate_);

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
            return segment_generator_.createTrajectory(start, goal, result);
        }

    } // namespace trajectory_generators


    namespace back_trackers {

        ModuleFactory::Registration <FeasibleRotateReverse> FeasibleRotateReverse::registration(
                "FeasibleRotateReverse");

        void FeasibleRotateReverse::setupFromParamMap(Module::ParamMap *param_map) {
            // Setup parent and segment creator
            RotateReverse::setupFromParamMap(param_map);
            segment_generator_.setConstraints(1.0, 1.0, turn_rate_, sampling_rate_);
        }

        bool FeasibleRotateReverse::rotate(TrajectorySegment *target) {
            mav_msgs::EigenTrajectoryPoint goal;
            goal.position_W = target->trajectory.back().position_W;
            goal.setFromYaw(defaults::angleScaled(target->trajectory.back().getYaw() + turn_rate_ / update_rate_));
            mav_msgs::EigenTrajectoryPointVector trajectory;
            if (segment_generator_.createTrajectory(target->trajectory.back(), goal, &trajectory)) {
                TrajectorySegment *new_segment = target->spawnChild();
                new_segment->trajectory = trajectory;
                current_rotation_ += turn_rate_ / update_rate_;
                last_yaw_ = new_segment->trajectory.back().getYaw();
                last_position_ = new_segment->trajectory.back().position_W;
                return true;
            } else {
                // if mav_trajectory_generation failed for some reason use the old implementation to not dir
                return RotateReverse::rotate(target);
            }
        }

    } // namespace back_trackers

}  // namespace mav_active_3d_planning