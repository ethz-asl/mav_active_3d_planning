#include "mav_active_3d_planning/modules/trajectory_evaluators/sensor_models/camera_model.h"
#include "mav_active_3d_planning/defaults.h"

#include <vector>
#include <algorithm>
#include <chrono>

namespace mav_active_3d_planning {
    namespace sensor_models {

        bool CameraModel::getVisibleVoxelsFromTrajectory(std::vector <Eigen::Vector3d> *result,
                                                         const TrajectorySegment &traj_in) {
            // Sample camera poses through trajectory
            std::vector<int> indices;
            sampleViewpoints(&indices, traj_in);

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

        void CameraModel::sampleViewpoints(std::vector<int> *result, const TrajectorySegment &traj_in) {
            if (p_sampling_time_ <= 0.0) {
                // Rate of 0 means only last point
                result->push_back(traj_in.trajectory.size() - 1);
            } else {
                int64_t sampling_time_ns = static_cast<int64_t>(p_sampling_time_ * 1.0e9);
                if (sampling_time_ns >= traj_in.trajectory.back().time_from_start_ns) {
                    // no points within one sampling interval: add last point
                    result->push_back(traj_in.trajectory.size() - 1);
                } else {
                    // sample the trajectory according to the sampling rate
                    int64_t current_time = sampling_time_ns;
                    for (int i = 0; i < traj_in.trajectory.size(); ++i) {
                        if (traj_in.trajectory[i].time_from_start_ns >= current_time) {
                            current_time += sampling_time_ns;
                            result->push_back(i);
                        }
                    }
                }
            }
        }

        void CameraModel::visualizeSensorView(visualization_msgs::MarkerArray *msg, const TrajectorySegment &traj_in) {
            std::vector<int> indices;
            sampleViewpoints(&indices, traj_in);

            for (int i = 0; i < indices.size(); ++i) {
                visualizeSingleView(msg, traj_in.trajectory[indices[i]].position_W,
                                    traj_in.trajectory[indices[i]].orientation_W_B);
            }
        }

        void CameraModel::visualizeSingleView(visualization_msgs::MarkerArray *msg, const Eigen::Vector3d &position,
                                              const Eigen::Quaterniond &orientation) {
            // Get boundary lines
            int res_x = ceil(p_ray_length_ * c_field_of_view_x_ / 0.25);
            int res_y = ceil(p_ray_length_ * c_field_of_view_y_ / 0.25);
            geometry_msgs::Point left[res_y + 1];
            geometry_msgs::Point right[res_y + 1];
            geometry_msgs::Point top[res_x + 1];
            geometry_msgs::Point bottom[res_x + 1];
            Eigen::Vector3d direction;

            for (int i = 0; i <= res_y; ++i) {
                getDirectionVector(&direction, 0.0, (double) i / (double) res_y);
                Eigen::Vector3d point = position + p_ray_length_ * (orientation * direction);
                left[i] = geometry_msgs::Point();
                left[i].x = point.x();
                left[i].y = point.y();
                left[i].z = point.z();
                getDirectionVector(&direction, 1.0, (double) i / (double) res_y);
                point = position + p_ray_length_ * (orientation * direction);
                right[i] = geometry_msgs::Point();
                right[i].x = point.x();
                right[i].y = point.y();
                right[i].z = point.z();
            }
            for (int i = 0; i <= res_x; ++i) {
                getDirectionVector(&direction, (double) i / (double) res_x, 0.0);
                Eigen::Vector3d point = position + p_ray_length_ * (orientation * direction);
                bottom[i] = geometry_msgs::Point();
                bottom[i].x = point.x();
                bottom[i].y = point.y();
                bottom[i].z = point.z();
                getDirectionVector(&direction, (double) i / (double) res_x, 1.0);
                point = position + p_ray_length_ * (orientation * direction);
                top[i] = geometry_msgs::Point();
                top[i].x = point.x();
                top[i].y = point.y();
                top[i].z = point.z();
            }

            // Build message
            visualization_msgs::Marker new_msg;
            new_msg.ns = "evaluation";
            new_msg.header.stamp = ros::Time::now();
            new_msg.header.frame_id = "/world";
            new_msg.id = defaults::getNextVisualizationId(*msg);
            new_msg.pose.orientation.w = 1.0;
            new_msg.type = visualization_msgs::Marker::LINE_LIST;
            new_msg.scale.x = 0.02;
            new_msg.color.r = 1.0;
            new_msg.color.g = 1.0;
            new_msg.color.b = 1.0;
            new_msg.color.a = 1.0;

            // points
            geometry_msgs::Point center;
            center.x = position.x();
            center.y = position.y();
            center.z = position.z();
            new_msg.points.push_back(center);
            new_msg.points.push_back(left[0]);
            new_msg.points.push_back(center);
            new_msg.points.push_back(left[res_y]);
            new_msg.points.push_back(center);
            new_msg.points.push_back(right[0]);
            new_msg.points.push_back(center);
            new_msg.points.push_back(right[res_y]);
            for (int i = 1; i <= res_x; ++i) {
                new_msg.points.push_back(top[i - 1]);
                new_msg.points.push_back(top[i]);
                new_msg.points.push_back(bottom[i - 1]);
                new_msg.points.push_back(bottom[i]);
            }
            for (int i = 1; i <= res_y; ++i) {
                new_msg.points.push_back(left[i - 1]);
                new_msg.points.push_back(left[i]);
                new_msg.points.push_back(right[i - 1]);
                new_msg.points.push_back(right[i]);
            }
            msg->markers.push_back(new_msg);
        }

        void CameraModel::getDirectionVector(Eigen::Vector3d *result, double relative_x, double relative_y) {
            *result = Eigen::Vector3d(p_focal_length_, (0.5 - relative_x) * (double) p_resolution_x_,
                                      (0.5 - relative_y) * (double) p_resolution_y_).normalized();
        }

        void CameraModel::setupFromParamMap(Module::ParamMap *param_map) {
            SensorModel::setupFromParamMap(param_map);
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