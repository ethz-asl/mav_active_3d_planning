#define _USE_MATH_DEFINES

#include "mav_active_3d_planning/modules/trajectory_generators/uniform.h"
#include "mav_active_3d_planning/defaults.h"

#include <mav_msgs/eigen_mav_msgs.h>

#include <random>
#include <cmath>

namespace mav_active_3d_planning {
    namespace trajectory_generators {

        ModuleFactory::Registration <Uniform> Uniform::registration("Uniform");

        void Uniform::setupFromParamMap(Module::ParamMap *param_map) {
            TrajectoryGenerator::setupFromParamMap(param_map);
            setParam<double>(param_map, "distance", &p_distance_, 1.0);
            setParam<double>(param_map, "yaw_angle", &p_yaw_angle_, 1.571);
            setParam<double>(param_map, "ascent_angle", &p_ascent_angle_, 0.523);
            setParam<double>(param_map, "p_sampling_rate", &p_sampling_rate_, 20.0);
            setParam<int>(param_map, "n_segments", &p_n_segments_, 5);

            c_yaw_rate_ = std::min(p_yaw_angle_ * system_constraints_->v_max / p_distance_ / 2.0, system_constraints_->yaw_rate_max);
        }

        bool Uniform::expandSegment(TrajectorySegment *target, std::vector<TrajectorySegment *> *new_segments) {
            // Create and add new adjacent trajectories to target segment
            target->tg_visited = true;
            int valid_segments = 0;
            TrajectorySegment *new_segment = target->spawnChild();
            int n_heights = 0;
            if (p_ascent_angle_ != 0.0) {
                n_heights = 2;
            }

            double current_ascent = 0.0;
            for (int j = -1; j < n_heights; ++j) {
                if (p_ascent_angle_ != 0.0) {
                    current_ascent = (double) j * p_ascent_angle_;
                }
                for (int i = 0; i < p_n_segments_; ++i) {
                    // Initialization
                    new_segment->trajectory.clear();
                    Eigen::Vector3d current_pos = target->trajectory.back().position_W;
                    double yaw_rate = ((double) i - (double) p_n_segments_ / 2.0 + 0.5) * c_yaw_rate_;
                    double current_yaw = target->trajectory.back().getYaw();
                    double current_distance = 0.0;
                    double current_time = 0.0;
                    bool collided = false;

                    while (current_distance < p_distance_) {
                        // Advance trajectory for every timestep
                        current_yaw += yaw_rate / p_sampling_rate_;
                        current_distance += system_constraints_->v_max / p_sampling_rate_;
                        current_time += 1.0 / p_sampling_rate_;
                        current_pos += system_constraints_->v_max / p_sampling_rate_ *
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
                        new_segments->push_back(new_segment);
                        new_segment = target->spawnChild();
                    }
                }
            }
            target->children.pop_back();

            // Feasible solution found?
            return (valid_segments > 0);
        }

    } // namespace trajectory_generators
}  // namespace mav_active_3d_planning