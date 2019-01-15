#include "mav_active_3d_planning/trajectory_generator.h"
#include "mav_active_3d_planning/trajectory_segment.h"
#include "mav_active_3d_planning/defaults.h"

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include <random>
#include <memory>

namespace mav_active_3d_planning {

    // Create random trajectories using the mav_trajectory_generation tools and check for MAV constraints (such as max
    // yaw rate, thrusts, ...)
    class TGMavTrajectoryGeneration: public TrajectoryGenerator {
    public:
        TGMavTrajectoryGeneration(voxblox::EsdfServer *voxblox_Ptr, bool p_coll_optimistic, double p_coll_radius)
                : TrajectoryGenerator(voxblox_Ptr, p_coll_optimistic, p_coll_radius) {}

        // Overwrite virtual functions
        bool expandSegment(TrajectorySegment &target);
        TrajectorySegment* selectSegment(TrajectorySegment &root);
        bool setParamsFromRos(const ros::NodeHandle &nh);

    protected:
        // parameters
        double p_distance_max_;
        double p_distance_min_;
        int p_n_segments_;
        int p_max_tries_;
        double p_uniform_weight_;   // [0-1] % of probability mass that is uniformly distributed

        mav_trajectory_generation::FeasibilityAnalytic feasibility_check_;
    };

    bool TGMavTrajectoryGeneration::setParamsFromRos(const ros::NodeHandle &nh) {
        nh.param("TG_distance_max", p_distance_max_, 3.0);
        nh.param("TG_distance_min", p_distance_min_, 0.5);
        nh.param("TG_n_segments", p_n_segments_, 5);
        nh.param("TG_max_tries", p_max_tries_, 100);
        nh.param("TG_uniform_weight", p_uniform_weight_, 0.2);

        // Create input constraints.for feasibility check
        // todo: set this from params
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
    }

    TrajectorySegment* TGMavTrajectoryGeneration::selectSegment(TrajectorySegment &root) {
        return defaults::selectRandomLeafWeighted(root, p_uniform_weight_);
    }

    bool TGMavTrajectoryGeneration::expandSegment(TrajectorySegment &target) {
        // Create and add new adjacent trajectories to target segment
        int valid_segments = 0;
        Eigen::Vector3d start_pos = target.trajectory.back().position_W;
        srand(time(NULL));
        int counter = 0;

        while (valid_segments < p_n_segments_ && counter < p_max_tries_) {
            counter++;
            // new random target
            double yaw = (double) rand() * 2.0 * M_PI / RAND_MAX;
            double theta = (double) rand() * M_PI / RAND_MAX;
            double range = p_distance_min_ + (double) rand() / RAND_MAX *(p_distance_max_-p_distance_min_);
            Eigen::Vector3d goal_pos = start_pos +  range *Eigen::Vector3d(sin(theta) * cos(yaw), sin(theta) * sin(yaw), cos(theta));

            // create trajectory
            mav_trajectory_generation::Vertex::Vector vertices;
            const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
            mav_trajectory_generation::Vertex start(4), goal(4);    // 4D trajectory
            Eigen::Vector4d start_4;
            start_4 << start_pos, target.trajectory.back().getYaw();
            start.makeStartOrEnd(start_4, derivative_to_optimize);
            void addConstraint(int type, const Eigen::VectorXd& constraint);
            vertices.push_back(start);
            Eigen::Vector4d goal_4;
            goal_4 << goal_pos, yaw;
            goal.makeStartOrEnd(goal_4, derivative_to_optimize);
            vertices.push_back(goal);

            const double v_max = 2.0;
            const double a_max = 2.0;
            std::vector<double> segment_times = mav_trajectory_generation::estimateSegmentTimes(vertices, v_max, a_max);

            mav_trajectory_generation::PolynomialOptimization<10> opt(4);   // Degree 10 polynomial for continuous snap
            opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
            opt.solveLinear();

            mav_trajectory_generation::Segment::Vector segments;
            opt.getSegments(&segments);

            // check input feasibility
            bool infeasible = false;
            for (int i=0; i< segments.size(); ++i) {
                if (feasibility_check_.checkInputFeasibility(segments[i]) !=
                        mav_trajectory_generation::InputFeasibilityResult::kInputFeasible) { infeasible=true; break; }
            }
            if (infeasible) { continue; }

            // convert to EigenTrajectory
            mav_trajectory_generation::Trajectory trajectory;
            opt.getTrajectory(&trajectory);
            mav_msgs::EigenTrajectoryPoint::Vector states;
            double sampling_interval = 0.1;     // sample at 10Hz
            if(!mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states)){ continue; }

            // Check collision
            bool collided = false;
            for (int i = 0; i < states.size(); ++i) {
                if (!checkCollision(states[i].position_W)) {
                    collided = true;
                    break;
                }
            }
            if (collided) { continue; }

            // Build result
            TrajectorySegment* new_segment = target.spawnChild();
            new_segment->trajectory = states;
            valid_segments++;
        }
        if (counter >= p_max_tries_ && valid_segments==0){
            // No feasible solution: try rotating only
            TrajectorySegment* new_segment = target.spawnChild();
            mav_msgs::EigenTrajectoryPoint trajectory_point;
            trajectory_point.position_W = start_pos;
            trajectory_point.setFromYaw((double) rand() * 2.0 * M_PI / (double) RAND_MAX);
            trajectory_point.time_from_start_ns = 0;
            new_segment->trajectory.push_back(trajectory_point);
            return false;
        }
        return true;
    }

}  // namespace mav_active_3d_planning