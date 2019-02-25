#define _USE_MATH_DEFINES

#include "mav_active_3d_planning/trajectory_generator.h"
#include "mav_active_3d_planning/defaults.h"

#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/param.h>

#include <random>
#include <cmath>

namespace mav_active_3d_planning {
    namespace trajectory_generators {

        // Point sampling in space, joined by linear segments
        class RandomLinear : public TrajectoryGenerator {
        public:
            RandomLinear(voxblox::EsdfServer *voxblox_ptr, std::string param_ns);

            // Overwrite virtual functions
            bool expandSegment(TrajectorySegment &target);

        protected:
            // params
            double p_min_distance_;     // m
            double p_max_distance_;     // m
            double p_v_max_;            // m/s
            double p_a_max_;            // m/s2
            double p_sampling_rate_;    // Hz
            int p_n_segments_;
            int p_max_tries_;
            bool p_planar_;
        };

        RandomLinear::RandomLinear(voxblox::EsdfServer *voxblox_ptr, std::string param_ns)
                : TrajectoryGenerator(voxblox_ptr, param_ns) {
            // Params
            ros::param::param<double>(param_ns + "/min_distance", p_min_distance_, 1.0);
            ros::param::param<double>(param_ns + "/max_distance", p_max_distance_, 1.0);
            ros::param::param<double>(param_ns + "/v_max", p_v_max_, 1.0);
            ros::param::param<double>(param_ns + "/a_max", p_a_max_, 1.0);
            ros::param::param<double>(param_ns + "/sampling_rate", p_sampling_rate_, 20.0);
            ros::param::param<int>(param_ns + "/n_segments", p_n_segments_, 5);
            ros::param::param<int>(param_ns + "/max_tries", p_max_tries_, 1000);
            ros::param::param<bool>(param_ns + "/planar", p_planar_, true);
        }

        bool RandomLinear::expandSegment(TrajectorySegment &target) {
            // Create and add new adjacent trajectories to target segment
            target.tg_visited = true;
            int valid_segments = 0;
            Eigen::Vector3d start_pos = target.trajectory.back().position_W;
            int counter = 0;
            TrajectorySegment *new_segment = target.spawnChild();

            while (valid_segments < p_n_segments_ && counter < p_max_tries_) {
                counter++;
                new_segment->trajectory.clear();
                // new random target direction
                double yaw = (double) rand() * 2.0 * M_PI / (double) RAND_MAX;
                double theta = (double) rand() * M_PI / (double) RAND_MAX;
                double distance = p_min_distance_ + (double) rand() / RAND_MAX * (p_max_distance_ - p_min_distance_);
                double decelleration_distance_ = distance - std::min(p_v_max_ * p_v_max_ / p_a_max_, distance) / 2;

                if (p_planar_) { theta = 0.5 * M_PI; }
                Eigen::Vector3d target_pos = start_pos + distance *
                                             Eigen::Vector3d(sin(theta) * cos(yaw), sin(theta) * sin(yaw), cos(theta));

                // Simulate trajectory (accelerate and deccelerate to 0 velocity at goal points)
                double x_curr = 0, v_curr = 0, t_curr = 0;
                bool collided = false;
                Eigen::Vector3d direction = (target_pos - start_pos).normalized();

                while (v_curr >= 0) {
                    if (x_curr < decelleration_distance_) {
                        v_curr = std::min(v_curr + p_a_max_ / p_sampling_rate_, p_v_max_);
                    } else {
                        v_curr -= p_a_max_ / p_sampling_rate_;
                    }
                    t_curr += 1.0 / p_sampling_rate_;
                    x_curr += v_curr / p_sampling_rate_;
                    Eigen::Vector3d current_pos = start_pos + x_curr * direction;

                    //check collision
                    if (!checkTraversable(current_pos)){
                        collided = true;
                        break;
                    }

                    // build result
                    mav_msgs::EigenTrajectoryPoint trajectory_point;
                    trajectory_point.position_W = current_pos;
                    trajectory_point.setFromYaw(defaults::angleScaled(yaw));
                    trajectory_point.time_from_start_ns = static_cast<int64_t>(t_curr * 1.0e9);
                    new_segment->trajectory.push_back(trajectory_point);
                }
                if (!collided){
                    valid_segments++;
                    new_segment = target.spawnChild();
                }
            }
            target.children.pop_back();
            // Feasible solution found?
            return (valid_segments > 0);
        }

    } // namespace trajectory_generators
}  // namespace mav_active_3d_planning