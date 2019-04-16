#define _USE_MATH_DEFINES

#include "mav_active_3d_planning/modules/trajectory_generators/feasible_rrt_star.h"

#include <vector>
#include <random>
#include <cmath>
#include <memory>

namespace mav_active_3d_planning {
    namespace trajectory_generators {

        ModuleFactory::Registration <FeasibleRRT> FeasibleRRT::registration("FeasibleRRT");

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
            return connectPosesFeasible(start, goal, result, system_constraints_->v_max,
                                        system_constraints_->a_max,
                                        system_constraints_->yaw_rate_max, p_sampling_rate_);
        }

        ModuleFactory::Registration <FeasibleRRTStar> FeasibleRRTStar::registration("FeasibleRRTStar");

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
            return FeasibleRRT::connectPosesFeasible(start, goal, result, system_constraints_->v_max,
                                                     system_constraints_->a_max,
                                                     system_constraints_->yaw_rate_max, p_sampling_rate_);
        }

        bool FeasibleRRT::connectPosesFeasible(const mav_msgs::EigenTrajectoryPoint &start,
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