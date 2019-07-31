#include "mav_active_3d_planning/modules/trajectory_generators/diffdrive_trajectory_generation.h"

#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <random>
#include <cmath>

namespace mav_active_3d_planning {
    namespace trajectory_generators {

        ModuleFactory::Registration <DiffDriveTrajectoryGeneration> DiffDriveTrajectoryGeneration::registration(
                "DiffDriveTrajectoryGeneration");

        void DiffDriveTrajectoryGeneration::setupFromParamMap(Module::ParamMap *param_map) {
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
            initializeConstraints();
        }

        bool DiffDriveTrajectoryGeneration::checkParamsValid(std::string *error_message) {
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

        void DiffDriveTrajectoryGeneration::initializeConstraints() {
            // Create input constraints.for feasibility check
            /*typedef mav_trajectory_generation::InputConstraintType ICT;
            mav_trajectory_generation::InputConstraints input_constraints;
            input_constraints.addConstraint(ICT::kFMin, 0.5 * 9.81); // minimum acceleration in [m/s/s].
            input_constraints.addConstraint(ICT::kFMax, 1.5 * 9.81); // maximum acceleration in [m/s/s].
            input_constraints.addConstraint(ICT::kVMax, system_constraints_->v_max); // maximum velocity in [m/s].
            input_constraints.addConstraint(ICT::kOmegaXYMax, M_PI / 2.0); // maximum roll/pitch rates in [rad/s].
            input_constraints.addConstraint(ICT::kOmegaZMax, system_constraints_->yaw_rate_max); // max yaw rate [rad/s].
            input_constraints.addConstraint(ICT::kOmegaZDotMax, M_PI); // maximum yaw acceleration in [rad/s/s]..
            feasibility_check_ = mav_trajectory_generation::FeasibilityAnalytic(input_constraints);
            feasibility_check_.settings_.setMinSectionTimeS(0.01);*/

            // Create nonlinear optimizer (Commented out since defaults should do)
//        parameters_.max_iterations = 1000;
//        parameters_.f_rel = 0.05;
//        parameters_.x_rel = 0.1;
//        parameters_.time_penalty = 500.0;
//        parameters_.initial_stepsize_rel = 0.1;
//        parameters_.inequality_constraint_tolerance = 0.1;

            parameters_ = mav_trajectory_generation::NonlinearOptimizationParameters();
        }

        bool DiffDriveTrajectoryGeneration::expandSegment(TrajectorySegment *target,
                                                    std::vector<TrajectorySegment *> *new_segments) {
            // Create and add new adjacent trajectories to target segment
            target->tg_visited = true;
            int valid_segments = 0;
            Eigen::Vector3d start_pos = target->trajectory.back().position_W;
            double start_vel_B_x = target->trajectory.back().velocity_W.squaredNorm();
            double start_yaw_W = std::acos(target->trajectory.back().orientation_W_B.w())*2; //because we know only yaw can be non zero
            int counter = 0;

            while (valid_segments < p_n_segments_ && counter < p_max_tries_) {
                counter++;
                //for now super simple, sample final velocity + yaw speed (need to later smooth somehow and stuff)
                //yaw jumps atm
                double dyaw = double(rand())/RAND_MAX * 2 * max_dyaw_ - max_dyaw_;
                double target_vel_B_x = double(rand())/RAND_MAX * 2 * max_vel_ - max_vel_;
                double distance = double(rand())/RAND_MAX * (p_distance_max_- p_distance_min_) + p_distance_min_;
                auto time = std::abs(distance / ((start_vel_B_x + target_vel_B_x) / 2));
                auto acc = (target_vel_B_x - start_vel_B_x) / time;
                if(std::abs(acc) > max_acc_ || time > max_time_) {
                    continue;
                }
                //std::cout << "new segment"<<std::endl;
                mav_msgs::EigenTrajectoryPoint::Vector states;
                auto currpos = start_pos; //getting position via super simple integration (1st order)
                for(int i = 0; i <= time * p_sampling_rate_; ++i){
                    auto time_from_start = double(i)/p_sampling_rate_;
                    
                    auto curryaw = start_yaw_W + dyaw * time_from_start;
                    auto currorientation_W_B = Eigen::Quaterniond(std::cos(curryaw/2), 0, 0, std::sin(curryaw/2));
                    auto currvel_W = currorientation_W_B * Eigen::Vector3d(start_vel_B_x + acc * time_from_start, 0, 0);
                    currpos += 1.0/p_sampling_rate_ * currvel_W;
                    mav_msgs::EigenTrajectoryPoint trajectory_point;
                    trajectory_point.position_W = currpos;
                    trajectory_point.setFromYaw(defaults::angleScaled(curryaw));
                    //std::cout << i << " gives " << static_cast<int64_t>(time_from_start * 1.0e9) << std::endl; seems ok
                    trajectory_point.time_from_start_ns = static_cast<int64_t>(time_from_start * 1.0e9);
                    states.push_back(trajectory_point);
                    /*states.push_back(
                        mav_msgs::EigenTrajectoryPoint(
                            time_from_start * 10e9,
                            currpos,
                            currvel_W,
                            currorientation_W_B * Eigen::Vector3d(acc, 0, 0),
                            Eigen::Vector3d(0, 0, 0), //jerk?? //some derivative stuff?
                            Eigen::Vector3d(0, 0, 0), //snap??
                            currorientation_W_B,
                            Eigen::Vector3d(0, 0, dyaw),
                            Eigen::Vector3d(0, 0, 0)
                        )
                    );*/
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


        bool DiffDriveTrajectoryGeneration::checkInputFeasible(const mav_trajectory_generation::Segment::Vector &/*segments*/) {
            /*for (int i = 0; i < segments.size(); ++i) {
                if (feasibility_check_.checkInputFeasibility(segments[i]) !=
                    mav_trajectory_generation::InputFeasibilityResult::kInputFeasible) {
                    return false;
                }
            }*/
            return true;
        }

        bool DiffDriveTrajectoryGeneration::checkTrajectoryCollision(const mav_msgs::EigenTrajectoryPoint::Vector &states) {
            for (int i = 0; i < states.size(); ++i) {
                if (!checkTraversable(states[i].position_W)) {
                    return false;
                }
            }
            return true;
        }

    } // namespace trajectory_generators
}  // namespace mav_active_3d_planning