#include "mav_active_3d_planning/ray_caster.h"

#include <ros/param.h>
#include <Eigen/Dense>

#include <vector>
#include <algorithm>
#include <chrono>

namespace mav_active_3d_planning {

    RayCaster::RayCaster(voxblox::EsdfServer *voxblox_ptr, std::string param_ns) : voxblox_ptr_(voxblox_ptr) {
        // Cache constants from voxblox
        c_voxel_size_ = voxblox_ptr_->getEsdfMapPtr()->voxel_size();
        c_block_size_ = voxblox_ptr_->getEsdfMapPtr()->block_size();

        // get params from ros
        std::string camera_params_ns;
        ros::param::param<std::string>(param_ns+"/camera_params_ns", camera_params_ns, param_ns);
        ros::param::param<double>(param_ns+"/ray_length", p_ray_length_, 5.0);
        ros::param::param<double>(param_ns+"/ray_step", p_ray_step_, static_cast<double>(c_voxel_size_));
        ros::param::param<double>(param_ns+"/sampling_time", p_sampling_time_, 0.0);

        // Setup implementation with default capturing (default should capture most recent/bets impl)
        ros::param::param<bool>(param_ns+"/raycaster_test", p_test_, false);
        std::string impl;
        ros::param::param<std::string>(param_ns+"/raycaster_implementation", impl, std::string(""));
        if (impl == "Simple"){
            impl_ = &mav_active_3d_planning::RayCaster::implSimple;
        } else {
            impl_ = &mav_active_3d_planning::RayCaster::implIterativeRaySampling;
        }

        // Wait until the camera params are on parameter server
        while (!ros::param::has(camera_params_ns+"/width")) { ros::Duration(0.1).sleep(); }
        ros::param::get(camera_params_ns+"/width", p_resolution_x_);
        ros::param::get(camera_params_ns+"/height", p_resolution_y_);
        ros::param::get(camera_params_ns+"/focal_length", p_focal_length_);

        // cache param dependent constants
        c_field_of_view_x_ = 2.0 * atan(p_resolution_x_ /  p_focal_length_ / 2.0);
        c_field_of_view_y_ = 2.0 * atan(p_resolution_y_ /  p_focal_length_ / 2.0);
        // Downsample to voxel size resolution at max range
        c_res_x_ = std::min((int)ceil(p_ray_length_ * c_field_of_view_x_ / (double)c_voxel_size_), p_resolution_x_);
        c_res_y_ = std::min((int)ceil(p_ray_length_ * c_field_of_view_y_ / (double)c_voxel_size_), p_resolution_y_);

    }

    std::vector<Eigen::Vector3d> RayCaster::getVisibleVoxels(Eigen::Vector3d position, Eigen::Quaterniond orientation) {
        if (!p_test_) {
            return (this->*impl_)(position, orientation);
        } else {
            auto start_time = std::chrono::high_resolution_clock::now();
            std::vector <Eigen::Vector3d> result = (this->*impl_)(position, orientation);
            auto end_time = std::chrono::high_resolution_clock::now();
            time_count_.push_back((double)((end_time - start_time)/std::chrono::milliseconds(1)));
            if (time_count_.size() % 20 == 0){
                double mean = 0.0;
                double stddev = 0.0;
                for (int i = 0; i<time_count_.size(); ++i){
                    mean += time_count_[i];
                }
                mean /= time_count_.size();
                for (int i = 0; i<time_count_.size(); ++i){
                    stddev += std::pow(time_count_[i]-mean, 2.0);
                }
                stddev = std::sqrt(stddev / time_count_.size());
                ROS_INFO("Raycasting: Mean: %.2f ns, Stddev: %.2f ns", mean, stddev);
            }
            return result;
        }
    }

    std::vector<Eigen::Vector3d> RayCaster::implSimple(Eigen::Vector3d position, Eigen::Quaterniond orientation) {
        std::vector <Eigen::Vector3d> result;
        // Naive ray-casting
        Eigen::Vector3d unit_direction = orientation * Eigen::Vector3d(1, 0, 0);
        for (int i = 0; i < c_res_x_; ++i) {
            Eigen::Vector3d x_direction = Eigen::AngleAxis<double>(c_field_of_view_x_ * ((double) i / c_res_x_ - 0.5),
                                                                   Eigen::Vector3d(0, 0, 1)) * unit_direction;
            for (int j = 0; j < c_res_y_; ++j) {
                Eigen::Vector3d direction = Eigen::AngleAxis<double>(c_field_of_view_y_ * ((double) j / c_res_y_ - 0.5),
                                                                     Eigen::Vector3d(0, 1, 0)) * x_direction;
                double distance = 0.0;
                while (distance < p_ray_length_) {
                    Eigen::Vector3d current_position = position + distance * direction;
                    distance += p_ray_step_;

                    // Check voxel occupied
                    double map_distance = 0.0;
                    if (voxblox_ptr_->getEsdfMapPtr()->getDistanceAtPosition(current_position, &map_distance)) {
                        if (map_distance < 0.0) { break; }
                    }

                    // Get voxel center
                    voxblox::BlockIndex block_id = voxblox_ptr_->getEsdfMapPtr()->getEsdfLayerPtr()->
                            computeBlockIndexFromCoordinates(current_position.cast<voxblox::FloatingPoint>());
                    Eigen::Vector3d center_point =
                            voxblox::getOriginPointFromGridIndex(block_id, c_block_size_).cast<double>();
                    voxblox::VoxelIndex voxel_id = voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
                            (current_position-center_point).cast<voxblox::FloatingPoint>(), 1.0 / c_voxel_size_);
                    center_point += voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size_).cast<double>();
                    result.push_back(center_point);
                }
            }
        }
        result.erase(std::unique(result.begin(), result.end()), result.end());
        return result;
    }

    std::vector<Eigen::Vector3d> RayCaster::implIterativeRaySampling(Eigen::Vector3d position, Eigen::Quaterniond orientation) {
        std::vector <Eigen::Vector3d> result;
        // Only cast rays until they span a single voxel, then duplicate

        // Determine number of splits + split distances
        int n_sections = (int) std::floor(std::log2(std::min((double) c_res_x_, (double) c_res_y_)));
        std::vector<double> split_distances;
        std::vector<int> split_widths;
        split_widths.push_back(0);
        for (int i=0; i<n_sections; ++i){
            split_widths.push_back(std::pow(2, i));
            split_distances.push_back(p_ray_length_ / std::pow(2.0, (double)i));
        }
        split_distances.push_back(0.0);
        std::reverse(split_distances.begin(),split_distances.end());
        std::reverse(split_widths.begin(),split_widths.end());

        // Setup ray table (contains where to start, -1 if occluded
        Eigen::ArrayXXi ray_table = Eigen::ArrayXXi::Zero(c_res_x_, c_res_y_);

        // Ray-casting
        Eigen::Vector3d unit_direction = orientation * Eigen::Vector3d(1, 0, 0);
        for (int i = 0; i < c_res_x_; ++i) {
            Eigen::Vector3d x_direction = Eigen::AngleAxis<double>(c_field_of_view_x_ * (0.5 - (double) i / c_res_x_),
                                                                   Eigen::Vector3d(0, 0, 1)) * unit_direction;
            for (int j = 0; j < c_res_y_; ++j) {
                int current_segment = ray_table(i, j);
                if (current_segment < 0) { continue; }
                Eigen::Vector3d direction = Eigen::AngleAxis<double>(c_field_of_view_y_ * (0.5 - (double) j / c_res_y_),
                                                                     Eigen::Vector3d(0, 1, 0)) * x_direction;

                double distance = split_distances[current_segment];
                bool cast_ray = true;
                while (cast_ray) {
                    while (distance < split_distances[current_segment + 1]) {
                        Eigen::Vector3d current_position = position + distance * direction;
                        distance += p_ray_step_;

                        // Get voxel center and add to result
                        voxblox::BlockIndex block_id = voxblox_ptr_->getEsdfMapPtr()->getEsdfLayerPtr()->
                                computeBlockIndexFromCoordinates(current_position.cast<voxblox::FloatingPoint>());
                        Eigen::Vector3d center_point =
                                voxblox::getOriginPointFromGridIndex(block_id, c_block_size_).cast<double>();
                        voxblox::VoxelIndex voxel_id = voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
                                (current_position - center_point).cast<voxblox::FloatingPoint>(), 1.0 / c_voxel_size_);
                        center_point += voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size_).cast<double>();
                        result.push_back(center_point);

                        // Check voxel occupied
                        double map_distance = 0.0;
                        if (voxblox_ptr_->getEsdfMapPtr()->getDistanceAtPosition(current_position, &map_distance)) {
                            if (map_distance < 0.0) {
                                // Occlusion
                                for (int i2 = i; i2 < std::min(c_res_x_, i + split_widths[current_segment]); ++i2){
                                    for (int j2 = j; j2 < std::min(c_res_y_, j + split_widths[current_segment]); ++j2){
                                        ray_table(i2, j2) = -1;
                                    }
                                }
                                cast_ray = false;
                                break;
                            }
                        }
                    }
                    if (cast_ray) {
                        current_segment++;
                        if (current_segment >= n_sections) {
                            cast_ray = false; // done
                        } else {
                            for (int i2 = i; i2 < std::min(c_res_x_, i + split_widths[current_segment - 1]); ++i2) {
                                for (int j2 = j; j2 < std::min(c_res_y_, j + split_widths[current_segment - 1]); ++j2) {
                                    ray_table(i2, j2) = current_segment;
                                }
                            }
                        }
                    }
                }
            }
        }
        result.erase(std::unique(result.begin(), result.end()), result.end());
        return result;
    }

    std::vector <Eigen::Vector3d> RayCaster::getVisibleVoxelsFromTrajectory(TrajectorySegment* traj_in) {
        // Sample camera poses through trajectory
        std::vector<int> indices;
        if (p_sampling_time_ <= 0.0) {
            // Rate of 0 means only last point
            indices.push_back(traj_in->trajectory.size() - 1);
        } else {
            int64_t sampling_time_ns = static_cast<int64_t>(p_sampling_time_ * 1.0e9);
            if (sampling_time_ns >= traj_in->trajectory.back().time_from_start_ns) {
                // no points within one sampling interval: add last point
                indices.push_back(traj_in->trajectory.size() - 1);
            } else {
                int64_t current_time = sampling_time_ns;
                for (int i = 0; i < traj_in->trajectory.size(); ++i) {
                    if (traj_in->trajectory[i].time_from_start_ns >= current_time) {
                        current_time += sampling_time_ns;
                        indices.push_back(i);
                    }
                }
            }
        }

        // Get all visible voxels
        std::vector <Eigen::Vector3d> new_voxels;
        for (int i=0; i < indices.size(); ++i ){
            std::vector <Eigen::Vector3d> this_view = getVisibleVoxels(
                    traj_in->trajectory[indices[i]].position_W, traj_in->trajectory[indices[i]].orientation_W_B);
            new_voxels.insert(new_voxels.begin(), this_view.begin(), this_view.end());
        }

        // Remove non-unique voxels
        new_voxels.erase( std::unique(new_voxels.begin(), new_voxels.end()), new_voxels.end() );
        return new_voxels;
    }

} // namepsace mav_active_3d_planning