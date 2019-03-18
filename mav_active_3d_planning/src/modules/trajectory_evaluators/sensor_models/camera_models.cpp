#include "mav_active_3d_planning/modules/trajectory_evaluators/sensor_models/camera_models.h"

#include <Eigen/Dense>

#include <vector>
#include <algorithm>
#include <chrono>

namespace mav_active_3d_planning {
    namespace sensor_models {

        // CameraModel
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
                getVisibleVoxels(result, traj_in.trajectory[indices[i]].position_W,
                                 traj_in.trajectory[indices[i]].orientation_W_B);
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
            return SensorModel::checkParamsValid(error_message);
        }

        // SimpleRayCaster
        void SimpleRayCaster::setupFromParamMap(Module::ParamMap *param_map) {
            CameraModel::setupFromParamMap(param_map);
            setParam<double>(param_map, "ray_step", &p_ray_step_, (double) c_voxel_size_);

            // Downsample to voxel size resolution at max range
            c_res_x_ = std::min((int) ceil(p_ray_length_ * c_field_of_view_x_ / (double) c_voxel_size_),
                                p_resolution_x_);
            c_res_y_ = std::min((int) ceil(p_ray_length_ * c_field_of_view_y_ / (double) c_voxel_size_),
                                p_resolution_y_);

        }

        bool SimpleRayCaster::getVisibleVoxels(std::vector <Eigen::Vector3d> *result, const Eigen::Vector3d &position,
                                               const Eigen::Quaterniond &orientation) {
            // Naive ray-casting
            Eigen::Vector3d unit_direction = orientation * Eigen::Vector3d(1, 0, 0);
            for (int i = 0; i < c_res_x_; ++i) {
                Eigen::Vector3d x_direction =
                        Eigen::AngleAxis<double>(c_field_of_view_x_ * ((double) i / c_res_x_ - 0.5),
                                                 Eigen::Vector3d(0, 0, 1)) * unit_direction;
                for (int j = 0; j < c_res_y_; ++j) {
                    Eigen::Vector3d direction =
                            Eigen::AngleAxis<double>(c_field_of_view_y_ * ((double) j / c_res_y_ - 0.5),
                                                     Eigen::Vector3d(0, 1, 0)) * x_direction;
                    double distance = 0.0;
                    while (distance < p_ray_length_) {
                        Eigen::Vector3d current_position = position + distance * direction;
                        distance += p_ray_step_;

                        // Add point (duplicates are handled in CameraModel::getVisibleVoxelsFromTrajectory)
                        getVoxelCenter(&current_position);
                        result->push_back(current_position);

                        // Check voxel occupied
                        double map_distance = 0.0;
                        if (voxblox_ptr_->getEsdfMapPtr()->getDistanceAtPosition(current_position, &map_distance)) {
                            if (map_distance < 0.0) { break; }
                        }
                    }
                }
            }
            return true;
        }

        // IterativeRayCaster
        void IterativeRayCaster::setupFromParamMap(Module::ParamMap *param_map) {
            CameraModel::setupFromParamMap(param_map);
            setParam<double>(param_map, "ray_step", &p_ray_step_, (double) c_voxel_size_);

            // Downsample to voxel size resolution at max range
            c_res_x_ = std::min((int) ceil(p_ray_length_ * c_field_of_view_x_ / (double) c_voxel_size_),
                                p_resolution_x_);
            c_res_y_ = std::min((int) ceil(p_ray_length_ * c_field_of_view_y_ / (double) c_voxel_size_),
                                p_resolution_y_);

            // Determine number of splits + split distances
            c_n_sections_ = (int) std::floor(std::log2(std::min((double) c_res_x_, (double) c_res_y_)));
            c_split_widths_.push_back(0);
            for (int i = 0; i < c_n_sections_; ++i) {
                c_split_widths_.push_back(std::pow(2, i));
                c_split_distances_.push_back(p_ray_length_ / std::pow(2.0, (double) i));
            }
            c_split_distances_.push_back(0.0);
            std::reverse(c_split_distances_.begin(), c_split_distances_.end());
            std::reverse(c_split_widths_.begin(), c_split_widths_.end());
        }

        bool IterativeRayCaster::getVisibleVoxels(std::vector <Eigen::Vector3d> *result, const Eigen::Vector3d &position,
                                                  const Eigen::Quaterniond &orientation) {
            // Setup ray table (contains at which segment to start, -1 if occluded
            ray_table_ = Eigen::ArrayXXi::Zero(c_res_x_, c_res_y_);

            // Ray-casting
            Eigen::Vector3d unit_direction = orientation * Eigen::Vector3d(1, 0, 0);
            for (int i = 0; i < c_res_x_; ++i) {
                Eigen::Vector3d x_direction =
                        Eigen::AngleAxis<double>(c_field_of_view_x_ * (0.5 - (double) i / c_res_x_),
                                                 Eigen::Vector3d(0, 0, 1)) * unit_direction;
                for (int j = 0; j < c_res_y_; ++j) {
                    int current_segment = ray_table_(i, j);
                    if (current_segment < 0) { continue; }
                    Eigen::Vector3d direction =
                            Eigen::AngleAxis<double>(c_field_of_view_y_ * (0.5 - (double) j / c_res_y_),
                                                     Eigen::Vector3d(0, 1, 0)) * x_direction;

                    double distance = c_split_distances_[current_segment];
                    bool cast_ray = true;
                    while (cast_ray) {
                        while (distance < c_split_distances_[current_segment + 1]) {
                            Eigen::Vector3d current_position = position + distance * direction;
                            distance += p_ray_step_;

                            // Add point (duplicates are handled in CameraModel::getVisibleVoxelsFromTrajectory)
                            getVoxelCenter(&current_position);
                            result->push_back(current_position);

                            // Check voxel occupied
                            double map_distance = 0.0;
                            if (voxblox_ptr_->getEsdfMapPtr()->getDistanceAtPosition(current_position, &map_distance)) {
                                if (map_distance < 0.0) {
                                    // Occlusion
                                    for (int i2 = i;
                                         i2 < std::min(c_res_x_, i + c_split_widths_[current_segment]); ++i2) {
                                        for (int j2 = j;
                                             j2 < std::min(c_res_y_, j + c_split_widths_[current_segment]); ++j2) {
                                            ray_table_(i2, j2) = -1;
                                        }
                                    }
                                    cast_ray = false;
                                    break;
                                }
                            }
                        }
                        if (cast_ray) {
                            current_segment++;
                            if (current_segment >= c_n_sections_) {
                                cast_ray = false; // done
                            } else {
                                // update ray starts
                                for (int i2 = i;
                                     i2 < std::min(c_res_x_, i + c_split_widths_[current_segment - 1]); ++i2) {
                                    for (int j2 = j;
                                         j2 < std::min(c_res_y_, j + c_split_widths_[current_segment - 1]); ++j2) {
                                        ray_table_(i2, j2) = current_segment;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            return true;
        }

        // Timing functionalities if other models are to be implemented / timed
        // TODO: remove in final version
//    std::vector<Eigen::Vector3d> RayCaster::getVisibleVoxels(Eigen::Vector3d position, Eigen::Quaterniond orientation) {
//        if (!p_test_) {
//            return (this->*impl_)(position, orientation);
//        } else {
//            auto start_time = std::chrono::high_resolution_clock::now();
//            std::vector <Eigen::Vector3d> result = (this->*impl_)(position, orientation);
//            auto end_time = std::chrono::high_resolution_clock::now();
//            time_count_.push_back((double)((end_time - start_time)/std::chrono::milliseconds(1)));
//            if (time_count_.size() % 20 == 0){
//                double mean = 0.0;
//                double stddev = 0.0;
//                for (int i = 0; i<time_count_.size(); ++i){
//                    mean += time_count_[i];
//                }
//                mean /= time_count_.size();
//                for (int i = 0; i<time_count_.size(); ++i){
//                    stddev += std::pow(time_count_[i]-mean, 2.0);
//                }
//                stddev = std::sqrt(stddev / time_count_.size());
//                ROS_INFO("Raycasting: Mean: %.2f ns, Stddev: %.2f ns", mean, stddev);
//            }
//            return result;
//        }
//    }

    } // namespace sensor_models
} // namepsace mav_active_3d_planning