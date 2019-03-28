#include "mav_active_3d_planning/modules/trajectory_evaluators/sensor_models/camera_model.h"

#include <vector>
#include <algorithm>
#include <chrono>

namespace mav_active_3d_planning {
    namespace sensor_models {

        bool CameraModel::getVisibleVoxelsFromTrajectory(std::vector <Eigen::Vector3d> *result,
                                                         const TrajectorySegment &traj_in) {
            // Sample camera poses through trajectory
            std::vector<int> indices;
            if (p_sampling_time_ <= 0.0) {
                // Rate of 0 means only last point
                indices.push_back(traj_in.trajectory.size() - 1);
            } else {
                int64_t sampling_time_ns = static_cast<int64_t>(p_sampling_time_ * 1.0e9);
                if (sampling_time_ns >= traj_in.trajectory.back().time_from_start_ns) {
                    // no points within one sampling interval: add last point
                    indices.push_back(traj_in.trajectory.size() - 1);
                } else {
                    // sample the trajectory according to the sampling rate
                    int64_t current_time = sampling_time_ns;
                    for (int i = 0; i < traj_in.trajectory.size(); ++i) {
                        if (traj_in.trajectory[i].time_from_start_ns >= current_time) {
                            current_time += sampling_time_ns;
                            indices.push_back(i);
                        }
                    }
                }
            }

            // Get all visible voxels
            for (int i = 0; i < indices.size(); ++i) {
                if (!p_test_) {
                    getVisibleVoxels(result, traj_in.trajectory[indices[i]].position_W,
                                     traj_in.trajectory[indices[i]].orientation_W_B);
                } else {
                    // Time the performance of raycasters
                    auto start_time = std::chrono::high_resolution_clock::now();
                    getVisibleVoxels(result, traj_in.trajectory[indices[i]].position_W,
                                     traj_in.trajectory[indices[i]].orientation_W_B);
                    auto end_time = std::chrono::high_resolution_clock::now();
                    time_count_.push_back((double) ((end_time - start_time) / std::chrono::milliseconds(1)));
                    if (time_count_.size() % 20 == 0) {
                        double mean = 0.0;
                        double stddev = 0.0;
                        for (int i = 0; i < time_count_.size(); ++i) {
                            mean += time_count_[i];
                        }
                        mean /= time_count_.size();
                        for (int i = 0; i < time_count_.size(); ++i) {
                            stddev += std::pow(time_count_[i] - mean, 2.0);
                        }
                        stddev = std::sqrt(stddev / time_count_.size());
                        ROS_INFO("Raycasting: Mean: %.2f ms, Stddev: %.2f ms", mean, stddev);
                    }
                }
            }

            // Remove non-unique voxels
            result->erase(std::unique(result->begin(), result->end()), result->end());
            return true;
        }

        void CameraModel::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<double>(param_map, "ray_length", &p_ray_length_, 5.0);
            setParam<double>(param_map, "sampling_time", &p_sampling_time_, 0.0);
            setParam<double>(param_map, "focal_length", &p_focal_length_, 320.0);
            setParam<int>(param_map, "resolution_x", &p_resolution_x_, 640);
            setParam<int>(param_map, "resolution_y", &p_resolution_y_, 480);
            setParam<bool>(param_map, "test", &p_test_, false); // set to measure performance of ray casters

            // cache param dependent constants
            c_field_of_view_x_ = 2.0 * atan2(p_resolution_x_, p_focal_length_ * 2.0);
            c_field_of_view_y_ = 2.0 * atan2(p_resolution_y_, p_focal_length_ * 2.0);
        }

        bool CameraModel::checkParamsValid(std::string *error_message) {
            if (p_ray_length_ <= 0.0) {
                *error_message = "ray_length expected > 0.0";
                return false;
            } else if (p_focal_length_ <= 0.0) {
                *error_message = "focal_length expected > 0.0";
                return false;
            } else if (p_resolution_x_ <= 0) {
                *error_message = "resolution_x expected > 0";
                return false;
            } else if (p_resolution_y_ <= 0.0) {
                *error_message = "resolution_y expected > 0";
                return false;
            }
            return true;
        }

    } // namespace sensor_models
} // namepsace mav_active_3d_planning