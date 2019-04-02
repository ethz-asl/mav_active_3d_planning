#define _USE_MATH_DEFINES

#include "mav_active_3d_planning/modules/back_trackers/default_back_trackers.h"
#include "mav_active_3d_planning/defaults.h"

#include <cmath>

namespace mav_active_3d_planning {
    namespace back_trackers {

        // RotateInPlace
        ModuleFactory::Registration<RotateInPlace> RotateInPlace::registration("RotateInPlace");

        bool RotateInPlace::trackBack(TrajectorySegment *target){
            // Just rotate the specified amount
            TrajectorySegment *new_segment = target->spawnChild();
            double yaw = target->trajectory.back().getYaw();
            for (int i = 0; i < (int)std::ceil(sampling_rate_ / update_rate_); ++i){
                mav_msgs::EigenTrajectoryPoint trajectory_point;
                trajectory_point.position_W = target->trajectory.back().position_W;
                yaw += turn_rate_ / sampling_rate_;
                trajectory_point.setFromYaw(defaults::angleScaled(yaw));
                trajectory_point.time_from_start_ns = static_cast<int64_t>((double)i / sampling_rate_ * 1.0e9);
                new_segment->trajectory.push_back(trajectory_point);
            }
            return true;
        }

        void RotateInPlace::setupFromParamMap(Module::ParamMap *param_map){
            setParam<double>(param_map, "turn_rate", &turn_rate_, 0.79);
            setParam<double>(param_map, "sampling_rate", &sampling_rate_, 20.0);
            setParam<double>(param_map, "update_rate", &update_rate_, 2.0);
        }

        bool RotateInPlace::checkParamsValid(std::string *error_message) {
            if (sampling_rate_ <= 0.0) {
                *error_message = "sampling_rate expected > 0";
                return false;
            } else if (update_rate_ <= 0.0) {
                *error_message = "update_rate expected > 0";
                return false;
            }
            return BackTracker::checkParamsValid(error_message);
        }

        // RotateReverse
        ModuleFactory::Registration<RotateReverse> RotateReverse::registration("RotateReverse");

         bool RotateReverse::segmentIsExecuted(const TrajectorySegment &segment) {
             // Don't track backtracking segments
             if (segment.trajectory.back().position_W == last_position_
                && segment.trajectory.back().getYaw() == last_yaw_ ) {
                return true;
            }
            // Register segments
             if (stack_.size() == stack_size_) {
                for (int i = 0; i < stack_size_ - 1; ++i) {
                    stack_[i] = stack_[i + 1];
                }
                 stack_[stack_size_ - 1] = segment.trajectory;
            } else {
                 stack_.push_back(segment.trajectory);
            }
            return true;
        }

        bool RotateReverse::trackBack(TrajectorySegment *target) {
            // Check wether this is the first backtracking step
            if (target->trajectory.back().position_W != last_position_
                || target->trajectory.back().getYaw() != last_yaw_ ) {
                current_rotation_ = 0.0;
            }
            if (current_rotation_ < n_rotations_ * 2.0 * M_PI) {
                // Rotate in place
                return rotate(target);
            } else {
                // Maximum rotations in place reached, time to reverse last segment
                if (stack_.empty()){
                    // Nothing to reverse: rotate again
                    current_rotation_ = 0.0;
                } else {
                    // Reverse last trajectory
                    return reverse(target);
                }
            }
        }

        bool RotateReverse::rotate(TrajectorySegment *target){
            // Rotate in place
            TrajectorySegment *new_segment = target->spawnChild();
            double yaw = target->trajectory.back().getYaw();
            for (int i = 0; i < (int) std::ceil(sampling_rate_ / update_rate_); ++i) {
                mav_msgs::EigenTrajectoryPoint trajectory_point;
                trajectory_point.position_W = target->trajectory.back().position_W;
                yaw += turn_rate_ / sampling_rate_;
                trajectory_point.setFromYaw(yaw);
                trajectory_point.time_from_start_ns =
                        static_cast<int64_t>((double) i / sampling_rate_ * 1.0e9);
                new_segment->trajectory.push_back(trajectory_point);
            }
            current_rotation_ += turn_rate_ / update_rate_;
            last_yaw_ = new_segment->trajectory.back().getYaw();
            last_position_ = new_segment->trajectory.back().position_W;
            return true;
        }
        bool RotateReverse::reverse(TrajectorySegment *target){
            // Reverse the last registered segment.
            mav_msgs::EigenTrajectoryPointVector to_reverse = stack_.back();
            std::reverse(to_reverse.begin(), to_reverse.end());
            int64_t current_time = 0;
            TrajectorySegment *new_segment = target->spawnChild();
            for (int i = 1; i < to_reverse.size(); ++i) {
                mav_msgs::EigenTrajectoryPoint trajectory_point;
                trajectory_point.position_W = to_reverse[i].position_W;
                trajectory_point.setFromYaw(defaults::angleScaled(to_reverse[i].getYaw() + M_PI));
                current_time += to_reverse[i-1].time_from_start_ns -
                                to_reverse[i].time_from_start_ns;
                trajectory_point.time_from_start_ns = current_time;
                new_segment->trajectory.push_back(trajectory_point);
            }
            last_yaw_ = new_segment->trajectory.back().getYaw();
            last_position_ = new_segment->trajectory.back().position_W;
            stack_.pop_back();
            return true;
        }

        void RotateReverse::setupFromParamMap(Module::ParamMap *param_map){
            setParam<double>(param_map, "turn_rate", &turn_rate_, 0.79);
            setParam<double>(param_map, "sampling_rate", &sampling_rate_, 20.0);
            setParam<double>(param_map, "update_rate", &update_rate_, 2.0);
            setParam<double>(param_map, "n_rotations", &n_rotations_, 2.0);
            setParam<int>(param_map, "stack_size", &stack_size_, 10);
            stack_.reserve(stack_size_);
        }

        bool RotateReverse::checkParamsValid(std::string *error_message) {
            if (sampling_rate_ <= 0.0) {
                *error_message = "sampling_rate expected > 0";
                return false;
            } else if (update_rate_ <= 0.0) {
                *error_message = "update_rate expected > 0";
                return false;
            } else if (stack_size_ <= 0) {
                *error_message = "stack_size expected > 0";
                return false;
            }
            return BackTracker::checkParamsValid(error_message);
        }

    } // namespace back_trackers
} // namepsace mav_active_3d_planning
