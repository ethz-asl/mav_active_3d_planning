#define _USE_MATH_DEFINES
#include "mav_active_3d_planning/trajectory_generator.h"
#include "mav_active_3d_planning/defaults.h"

#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/param.h>

#include <random>
#include <cmath>

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
            double p_yaw_angle_;        // rad
            double p_sampling_rate_;    // Hz
            int p_n_segments_;
            double p_ascent_angle_;     // Rad, Set 0 for planar
            double c_yaw_rate_;
        };

        Uniform::Uniform(voxblox::EsdfServer *voxblox_ptr, std::string param_ns)
                : TrajectoryGenerator(voxblox_ptr, param_ns) {
            // params
            ros::param::param<double>(param_ns + "/distance", p_distance_, 1.0);
            ros::param::param<double>(param_ns + "/velocity", p_velocity_, 0.33);
            ros::param::param<double>(param_ns + "/yaw_angle", p_yaw_angle_, 1.571);
            ros::param::param<double>(param_ns + "/ascent_angle", p_ascent_angle_, 0.523);
            ros::param::param<double>(param_ns + "/p_sampling_rate", p_sampling_rate_, 20.0);
            ros::param::param<int>(param_ns + "/n_segments", p_n_segments_, 5);

            c_yaw_rate_ = p_yaw_angle_ * p_velocity_ / p_distance_ / 2.0;
        }

        bool Uniform::expandSegment(TrajectorySegment &target) {
            // Create and add new adjacent trajectories to target segment
            target.tg_visited = true;
            int valid_segments = 0;
            TrajectorySegment *new_segment = target.spawnChild();
            int n_heights = 0;
            if (p_ascent_angle_ != 0.0) {
                n_heights = 2;
            }

            double current_ascent = 0.0;
            for (int j = -1; j < n_heights; ++j) {
                if (p_ascent_angle_ != 0.0) {
                    current_ascent = (double)j * p_ascent_angle_;
                }
                for (int i = 0; i < p_n_segments_; ++i) {
                    // Initialization
                    new_segment->trajectory.clear();
                    Eigen::Vector3d current_pos = target.trajectory.back().position_W;
                    double yaw_rate = ((double) i - (double) p_n_segments_ / 2.0 + 0.5) * c_yaw_rate_;
                    double current_yaw = target.trajectory.back().getYaw();
                    double current_distance = 0.0;
                    double current_time = 0.0;
                    bool collided = false;

                    while (current_distance < p_distance_) {
                        // Advance trajectory for every timestep
                        current_yaw += yaw_rate / p_sampling_rate_;
                        current_distance += p_velocity_ / p_sampling_rate_;
                        current_time += 1.0 / p_sampling_rate_;
                        current_pos += p_velocity_ / p_sampling_rate_ *
                                Eigen::Vector3d(cos(current_yaw) * cos(current_ascent),
                                                sin(current_yaw) * cos(current_ascent), sin(current_ascent));
                        if (!checkTraversable(current_pos)) {
                            collided = true;
                            break;
                        }
                        mav_msgs::EigenTrajectoryPoint trajectory_point;
                        trajectory_point.position_W = current_pos;
                        trajectory_point.setFromYaw(defaults::angleScaled(current_yaw));
                        trajectory_point.time_from_start_ns = static_cast<int64_t>(current_time * 1.0e9);
                        new_segment->trajectory.push_back(trajectory_point);
                    }
                    if (!collided) {
                        valid_segments++;
                        new_segment = target.spawnChild();
                    }
                }
            }
            target.children.pop_back();
            // Feasible solution found?
            return (valid_segments > 0);
        }

    } // namespace trajectory_generators
}  // namespace mav_active_3d_planning