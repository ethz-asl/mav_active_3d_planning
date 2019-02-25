#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/module_factory.h"

#include <ros/param.h>

#include <vector>
#include <chrono>

namespace mav_active_3d_planning {

    TrajectoryEvaluator::TrajectoryEvaluator(voxblox::EsdfServer *voxblox_ptr, std::string param_ns)
            : voxblox_ptr_(voxblox_ptr),
              bounding_volume_(param_ns + "/bounding_volume"),
              cost_computer_(nullptr),
              value_computer_(nullptr),
              next_selector_(nullptr),
              evaluator_updater_(nullptr),
              p_namespace_(param_ns) {}

    bool TrajectoryEvaluator::computeCost(TrajectorySegment &traj_in) {
        // If not implemented use a (default) module
        if (cost_computer_ == nullptr){
            cost_computer_ = ModuleFactory::createCostComputer(p_namespace_+"/cost_computer");
        }
        return cost_computer_->computeCost(traj_in);
    }

    bool TrajectoryEvaluator::computeValue(TrajectorySegment &traj_in) {
        // If not implemented use a (default) module
        if (value_computer_ == nullptr){
            value_computer_ = ModuleFactory::createValueComputer(p_namespace_+"/value_computer");
        }
        return value_computer_->computeValue(traj_in);
    }

    int TrajectoryEvaluator::selectNextBest(TrajectorySegment &traj_in) {
        // If not implemented use a (default) module
        if (next_selector_ == nullptr){
            next_selector_ = ModuleFactory::createNextSelector(p_namespace_+"/next_selector");
        }
        return next_selector_->selectNextBest(traj_in);
    }

    bool TrajectoryEvaluator::updateSegments(TrajectorySegment &root) {
        // If not implemented use a (default) module
        if (evaluator_updater_ == nullptr){
            evaluator_updater_ = ModuleFactory::createEvaluatorUpdater(p_namespace_+"/evaluator_updater", this);
        }
        return evaluator_updater_->updateSegments(root);
    }

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

        // Wait until the camera params are on parameter server
        while (!ros::param::has(camera_params_ns+"/width")) { ros::Duration(0.1).sleep(); }
        ros::param::get(camera_params_ns+"/width", p_resolution_x_);
        ros::param::get(camera_params_ns+"/height", p_resolution_y_);
        ros::param::get(camera_params_ns+"/focal_length", p_focal_length_);

        // cache param dependent constants
        c_field_of_view_x_ = 2.0 * atan(p_resolution_x_ /  p_focal_length_ / 2.0);
        c_field_of_view_y_ = 2.0 * atan(p_resolution_y_ /  p_focal_length_ / 2.0);
    }

    std::vector<Eigen::Vector3d> RayCaster::getVisibleVoxels(Eigen::Vector3d position, Eigen::Quaterniond orientation) {
        std::vector <Eigen::Vector3d> result;
//        auto start_time = std::chrono::high_resolution_clock::now();

        // Downsample to voxel size resolution at max range
        int res_x = std::min((int)ceil(p_ray_length_ * c_field_of_view_x_ / (double)c_voxel_size_), p_resolution_x_);
        int res_y = std::min((int)ceil(p_ray_length_ * c_field_of_view_y_ / (double)c_voxel_size_), p_resolution_y_);

        // Naive ray-casting
        Eigen::Vector3d unit_direction = orientation * Eigen::Vector3d(1, 0, 0);
        for (int i = 0; i < res_x; ++i) {
            Eigen::Vector3d x_direction = Eigen::AngleAxis<double>(c_field_of_view_x_ * ((double) i / res_x - 0.5),
                                                                   Eigen::Vector3d(0, 0, 1)) * unit_direction;
            for (int j = 0; j < res_y; ++j) {
                Eigen::Vector3d direction = Eigen::AngleAxis<double>(c_field_of_view_y_ * ((double) j / res_y - 0.5),
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
//        auto end_time = std::chrono::high_resolution_clock::now();
//        ROS_DEBUG("Raycasting: found %i voxels in %.3f miliseconds", (int)result.size(), (double)((end_time - start_time)/std::chrono::milliseconds(1)));
        return result;
    }

    std::vector <Eigen::Vector3d> RayCaster::getVisibleVoxelsFromTrajectory(TrajectorySegment* traj_in) {
        // Sample camera poses
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