#include "mav_active_3d_planning/trajectory_generator.h"

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <ros/param.h>

#include <random>

namespace mav_active_3d_planning {
    namespace trajectory_generators {

        // Create random trajectories using the mav_trajectory_generation tools and check for MAV constraints (such as max
        // yaw rate, thrusts, ...)
        class MavTrajectoryGeneration : public TrajectoryGenerator {
        public:
            MavTrajectoryGeneration(voxblox::EsdfServer *voxblox_ptr, std::string param_ns);

            // Overwrite virtual functions
            bool expandSegment(TrajectorySegment &target);

        protected:
            // parameters
            double p_distance_max_;
            double p_distance_min_;
            double p_v_max_;
            double p_a_max_;
            int p_n_segments_;
            int p_max_tries_;

            // optimization settings
            mav_trajectory_generation::FeasibilityAnalytic feasibility_check_;
            mav_trajectory_generation::NonlinearOptimizationParameters parameters_;
        };

        MavTrajectoryGeneration::MavTrajectoryGeneration(voxblox::EsdfServer *voxblox_ptr, std::string param_ns)
                : TrajectoryGenerator(voxblox_ptr, param_ns) {
            // Params
            ros::param::param<double>(param_ns + "/distance_max", p_distance_max_, 3.0);
            ros::param::param<double>(param_ns + "/distance_min", p_distance_min_, 0.5);
            ros::param::param<double>(param_ns + "/a_max", p_a_max_, 1.0);
            ros::param::param<double>(param_ns + "/v_max", p_v_max_, 1.0);
            ros::param::param<int>(param_ns + "/n_segments", p_n_segments_, 5);
            ros::param::param<int>(param_ns + "/max_tries", p_max_tries_, 100);

            // Create input constraints.for feasibility check
            // todo: set this from params or config
            typedef mav_trajectory_generation::InputConstraintType ICT;
            mav_trajectory_generation::InputConstraints input_constraints;
            input_constraints.addConstraint(ICT::kFMin, 0.5 * 9.81); // minimum acceleration in [m/s/s].
            input_constraints.addConstraint(ICT::kFMax, 1.5 * 9.81); // maximum acceleration in [m/s/s].
            input_constraints.addConstraint(ICT::kVMax, 3.5); // maximum velocity in [m/s].
            input_constraints.addConstraint(ICT::kOmegaXYMax, M_PI / 2.0); // maximum roll/pitch rates in [rad/s].
            input_constraints.addConstraint(ICT::kOmegaZMax, M_PI / 2.0); // maximum yaw rates in [rad/s].
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

        bool MavTrajectoryGeneration::expandSegment(TrajectorySegment &target) {
            // Create and add new adjacent trajectories to target segment
            target.tg_visited = true;
            int valid_segments = 0;
            Eigen::Vector3d start_pos = target.trajectory.back().position_W;
            srand(time(NULL));
            int counter = 0;

            while (valid_segments < p_n_segments_ && counter < p_max_tries_) {
                counter++;
                // new random target
                double yaw = (double) rand() * 2.0 * M_PI / RAND_MAX;
                double theta = (double) rand() * M_PI / RAND_MAX;
                double range = p_distance_min_ + (double) rand() / RAND_MAX * (p_distance_max_ - p_distance_min_);
                Eigen::Vector3d goal_pos =
                        start_pos + range * Eigen::Vector3d(sin(theta) * cos(yaw), sin(theta) * sin(yaw), cos(theta));

                // create trajectory (4D)
                Eigen::Vector4d start4, goal4, vel4, acc4;
                start4 << start_pos, target.trajectory.back().getYaw();
                goal4 << goal_pos, yaw;
                vel4 << target.trajectory.back().velocity_W, target.trajectory.back().getYawRate();
                acc4 << target.trajectory.back().acceleration_W, target.trajectory.back().getYawAcc();

                mav_trajectory_generation::Vertex::Vector vertices;
                const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;
                mav_trajectory_generation::Vertex start(4), goal(4);
                start.makeStartOrEnd(start4, derivative_to_optimize);
                start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, vel4);
                start.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, acc4);
                vertices.push_back(start);
                goal.makeStartOrEnd(goal4, derivative_to_optimize);

                // test: add some goal velocity
                Eigen::Vector4d constraint4;
                constraint4 << 0.5 * (goal_pos - start_pos).normalized(), 0;
                goal.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, constraint4);
                vertices.push_back(goal);

                // Optimize
                std::vector<double> segment_times = mav_trajectory_generation::estimateSegmentTimes(vertices, p_v_max_,
                                                                                                    p_a_max_);
                mav_trajectory_generation::PolynomialOptimizationNonLinear<10> opt(4, parameters_);
                opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
                opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, p_v_max_);
                opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, p_a_max_);
                opt.optimize();

                mav_trajectory_generation::Segment::Vector segments;
                opt.getPolynomialOptimizationRef().getSegments(&segments);

                // check input feasibility
                bool infeasible = false;
                for (int i = 0; i < segments.size(); ++i) {
                    if (feasibility_check_.checkInputFeasibility(segments[i]) !=
                        mav_trajectory_generation::InputFeasibilityResult::kInputFeasible) {
                        infeasible = true;
                        break;
                    }
                }
                if (infeasible) { continue; }

                // convert to EigenTrajectory
                mav_trajectory_generation::Trajectory trajectory;
                opt.getTrajectory(&trajectory);
                mav_msgs::EigenTrajectoryPoint::Vector states;
                double sampling_interval = 1.0 / 20;     // sample at 20Hz
                if (!mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval,
                                                                      &states)) { continue; }

                // Check collision
                bool collided = false;
                for (int i = 0; i < states.size(); ++i) {
                    if (!checkTraversable(states[i].position_W)) {
                        collided = true;
                        break;
                    }
                }
                if (collided) { continue; }

                // Build result
                TrajectorySegment *new_segment = target.spawnChild();
                new_segment->trajectory = states;
                valid_segments++;
            }
            // Feasible solution found?
            return (valid_segments > 0);
        }
    } // namespace trajectory_generators
}  // namespace mav_active_3d_planning