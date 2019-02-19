#define _USE_MATH_DEFINES
#include "mav_active_3d_planning/back_tracker.h"

#include <ros/param.h>
#include <ros/console.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <vector>
#include <cmath>

namespace mav_active_3d_planning {
    namespace back_trackers {

        // Try rotating in place until trajectories were found
        class Rotate : public BackTracker {
        public:
            Rotate(std::string param_ns) {
                ros::param::param<double>(param_ns + "/turn_rate", turn_rate_, 0.79);
                ros::param::param<double>(param_ns + "/sampling_rate", sampling_rate_, 2.0);
                ros::param::param<double>(param_ns + "/sampling_rate", sampling_rate_, 20.0);
            }

            bool trackBack(TrajectorySegment &target){
                // Just rotate the specified amount
                TrajectorySegment *new_segment = target.spawnChild();
                double yaw = target.trajectory.back().getYaw();
                for (int i = 0; i < (int)std::ceil(sampling_rate_ / update_rate_); ++i){
                    mav_msgs::EigenTrajectoryPoint trajectory_point;
                    trajectory_point.position_W = target.trajectory.back().position_W;
                    yaw += turn_rate_ / sampling_rate_;
                    trajectory_point.setFromYaw(yaw);
                    trajectory_point.time_from_start_ns = static_cast<int64_t>((double)i / sampling_rate_ * 1.0e9);
                    new_segment->trajectory.push_back(trajectory_point);
                }
            }

        protected:
            double turn_rate_;  // rad
            double update_rate_;   // Hz
            double sampling_rate_;  // Hz
        };

        // Try rotating in place, if nothing found reverse most recent segments
        class RotateReverse : public BackTracker {
        public:
            RotateReverse(std::string param_ns) {
                ros::param::param<double>(param_ns + "/turn_rate", turn_rate_, 0.79);
                ros::param::param<double>(param_ns + "/update_rate", update_rate_, 2.0);
                ros::param::param<double>(param_ns + "/sampling_rate", sampling_rate_, 20.0);
                ros::param::param<double>(param_ns + "/n_rotations", n_rotations_, 2.0);
                ros::param::param<int>(param_ns + "/stack_size", stack_size_, 10);

                stack_.reserve(stack_size_);
            }

             bool segmentIsExecuted(TrajectorySegment segment) {
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

            bool trackBack(TrajectorySegment &target) {
                // Check wether this is the first backtracking step
                if (target.trajectory.back().position_W != last_position_
                    || target.trajectory.back().getYaw() != last_yaw_ ) {
                    current_rotation_ = 0.0;
                }
                if (current_rotation_ > n_rotations_ * M_PI) {
                    if (stack_.empty()){
                        ROS_INFO("Backtracker: No trajectories to reverse, rotating again.");
                        current_rotation_ = 0.0;
                    } else {
                        // Reverse last trajectory
                        mav_msgs::EigenTrajectoryPointVector to_reverse = stack_.back();
                        int size = to_reverse.size() - 1;
                        int64_t current_time = 0;
                        TrajectorySegment *new_segment = target.spawnChild();
                        for (int i = 0; i <= size; ++i) {
                            mav_msgs::EigenTrajectoryPoint trajectory_point;
                            trajectory_point.position_W = to_reverse[size - i].position_W;
                            trajectory_point.setFromYaw(to_reverse[size - i].getYaw() + M_PI);
                            trajectory_point.time_from_start_ns = current_time;
                            current_time += to_reverse[size - i].time_from_start_ns -
                                            to_reverse[size - i - 1].time_from_start_ns;
                            new_segment->trajectory.push_back(trajectory_point);
                        }
                        last_yaw_ = new_segment->trajectory.back().getYaw();
                        last_position_ = new_segment->trajectory.back().position_W;
                        stack_.pop_back();
                        return true;
                    }
                }
                // Just rotate
                TrajectorySegment *new_segment = target.spawnChild();
                double yaw = target.trajectory.back().getYaw();
                for (int i = 0; i < (int)std::ceil(sampling_rate_ / update_rate_); ++i){
                    mav_msgs::EigenTrajectoryPoint trajectory_point;
                    trajectory_point.position_W = target.trajectory.back().position_W;
                    yaw += turn_rate_ / sampling_rate_;
                    trajectory_point.setFromYaw(yaw);
                    trajectory_point.time_from_start_ns = static_cast<int64_t>((double)i / sampling_rate_ * 1.0e9);
                    new_segment->trajectory.push_back(trajectory_point);
                }
                current_rotation_ += turn_rate_ / update_rate_;
                last_yaw_ = new_segment->trajectory.back().getYaw();
                last_position_ = new_segment->trajectory.back().position_W;
                return true;
            }

        protected:
            // params
            double turn_rate_;  // rad
            double update_rate_;   // Hz
            double sampling_rate_;  // Hz
            double n_rotations_;
            int stack_size_;

            // variables
            std::vector<mav_msgs::EigenTrajectoryPointVector> stack_;
            Eigen::Vector3d last_position_;
            double last_yaw_;
            double current_rotation_;
        };

    } // namespace back_trackers
} // namepsace mav_active_3d_planning
