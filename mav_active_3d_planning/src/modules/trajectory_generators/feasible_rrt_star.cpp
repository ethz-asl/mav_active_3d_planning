#define _USE_MATH_DEFINES

#include "mav_active_3d_planning/modules/trajectory_generators/feasible_rrt_star.h"

#include <vector>
#include <random>
#include <cmath>
#include <memory>

namespace mav_active_3d_planning {

    namespace trajectory_generators {

        ModuleFactory::Registration <FeasibleRRT> FeasibleRRT::registration("FeasibleRRT");

        void FeasibleRRT::setupFromParamMap(Module::ParamMap *param_map) {
            // Setup parent and segment creator
            RRT::setupFromParamMap(param_map);
            setParam<bool>(param_map, "all_semgents_feasible", &p_all_semgents_feasible_, false);
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
            if (p_all_semgents_feasible_) {
                return segment_generator_.createTrajectory(start, goal, result);
            } else {
                return segment_generator_.simulateTrajectory(start, goal, result);
            }
        }

        bool FeasibleRRT::extractTrajectoryToPublish(mav_msgs::EigenTrajectoryPointVector *trajectory,
                                                     const TrajectorySegment &segment) {
            if (p_all_semgents_feasible_) {
                // All segments are already feasible
                *trajectory = segment.trajectory;
                return true;
            } else {
                // Create a smooth semgent for execution
                return segment_generator_.createTrajectory(segment.trajectory.front(), segment.trajectory.back(),
                                                           trajectory);
            }
        }

        ModuleFactory::Registration <FeasibleRRTStar> FeasibleRRTStar::registration("FeasibleRRTStar");

        void FeasibleRRTStar::setupFromParamMap(Module::ParamMap *param_map) {
            // Setup parent and segment creator
            RRTStar::setupFromParamMap(param_map);
            setParam<bool>(param_map, "all_semgents_feasible", &p_all_semgents_feasible_, false);
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
            if (p_all_semgents_feasible_) {
                return segment_generator_.createTrajectory(start, goal, result);
            } else {
                return segment_generator_.simulateTrajectory(start, goal, result);
            }
        }

        bool FeasibleRRTStar::extractTrajectoryToPublish(mav_msgs::EigenTrajectoryPointVector *trajectory,
                                                         const TrajectorySegment &segment) {
            if (p_all_semgents_feasible_) {
                // All segments are already feasible
                *trajectory = segment.trajectory;
                return true;
            } else {
                // Create a smooth semgent for execution
                if (segment_generator_.createTrajectory(segment.trajectory.front(), segment.trajectory.back(),
                                                           trajectory)) {
                    return true;
                } else {
                    *trajectory = segment.trajectory;
                    return false;
                }
            }
        }

    } // namespace trajectory_generators

    namespace back_trackers {

        ModuleFactory::Registration <FeasibleRotateReverse> FeasibleRotateReverse::registration(
                "FeasibleRotateReverse");

        void FeasibleRotateReverse::setupFromParamMap(Module::ParamMap *param_map) {
            // Setup parent and segment creator
            RotateReverse::setupFromParamMap(param_map);
            segment_generator_.setConstraints(1.0, 1.0, turn_rate_, sampling_rate_);    // velocities don't matter
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
                // if mav_trajectory_generation failed for some reason use the old implementation to not die
                return RotateReverse::rotate(target);
            }
        }

    } // namespace back_trackers

}  // namespace mav_active_3d_planning