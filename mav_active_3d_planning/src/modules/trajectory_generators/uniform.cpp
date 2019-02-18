#include "mav_active_3d_planning/trajectory_generator.h"

#include <mav_msgs/eigen_mav_msgs.h>
#include <random>

namespace mav_active_3d_planning {
    namespace trajectory_generators {

        class Uniform : public TrajectoryGenerator {
        public:
            Uniform(voxblox::EsdfServer *voxblox_ptr, std::string param_ns);

            // Overwrite virtual functions
            bool expandSegment(TrajectorySegment &target);

        protected:
            // parameters
            double p_distance_;         // m
            double p_velocity_;         // m/s
            double p_yaw_rate_max_;     // rad/s
            double p_sampling_rate_;    // Hz
            int p_n_segments_;
            bool p_planar_;             // Doesnt do anything as yet
        };

        Uniform::Uniform(voxblox::EsdfServer *voxblox_ptr, std::string param_ns)
                : TrajectoryGenerator(voxblox_ptr, param_ns) {
            // params
            ros::param::param<double>(param_ns + "/distance", p_distance_, 1.0);
            ros::param::param<double>(param_ns + "/velocity", p_velocity_, 0.5);
            ros::param::param<double>(param_ns + "/yaw_rate_max", p_yaw_rate_max_, 1.7);
            ros::param::param<double>(param_ns + "/p_sampling_rate", p_sampling_rate_, 20.0);
            ros::param::param<int>(param_ns + "/n_segments", p_n_segments_, 5);
            ros::param::param<bool>(param_ns + "/planar", p_planar_, true);
        }

        bool Uniform::expandSegment(TrajectorySegment &target) {
            // Create and add new adjacent trajectories to target segment
            int valid_segments = 0;
            TrajectorySegment *new_segment = target.spawnChild();

            for (int i = 0; i < p_n_segments_; ++i) {
                // Initialization
                new_segment->trajectory.clear();
                Eigen::Vector3d current_pos = target.trajectory.back().position_W;
                double yaw_rate = ((double) i - (double) p_n_segments_ / 2.0 + 0.5) * p_yaw_rate_max_;
                double current_yaw = target.trajectory.back().getYaw();
                double current_distance = 0.0;
                double current_time = 0.0;
                bool collided = false;

                while (current_distance < p_distance_) {
                    // Advance trajectory for every timestep
                    current_yaw += yaw_rate / p_sampling_rate_;
                    current_distance += p_velocity_ / p_sampling_rate_;
                    current_time += 1.0 / p_sampling_rate_;
                    current_pos +=
                            p_velocity_ / p_sampling_rate_ * Eigen::Vector3d(cos(current_yaw), sin(current_yaw), 0.0);
                    if (!checkTraversable(current_pos)) {
                        collided = true;
                        break;
                    }
                    mav_msgs::EigenTrajectoryPoint trajectory_point;
                    trajectory_point.position_W = current_pos;
                    trajectory_point.setFromYaw(current_yaw);
                    trajectory_point.time_from_start_ns = static_cast<int64_t>(current_time * 1.0e9);
                    new_segment->trajectory.push_back(trajectory_point);
                }
                if (!collided) {
                    valid_segments++;
                    new_segment = target.spawnChild();
                }
            }
            if (valid_segments == 0) {
                // No feasible solution: try rotating only
                mav_msgs::EigenTrajectoryPoint trajectory_point;
                trajectory_point.position_W = target.trajectory.back().position_W;
                trajectory_point.setFromYaw(target.trajectory.back().getYaw() + 0.05);
                trajectory_point.time_from_start_ns = 0;
                new_segment->trajectory.clear();
                new_segment->trajectory.push_back(trajectory_point);
                return false;
            } else {
                target.children.pop_back();
                return true;
            }
        }
    } // namespace trajectory_generators
}  // namespace mav_active_3d_planning