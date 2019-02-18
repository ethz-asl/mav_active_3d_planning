#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/module_factory.h"

#include <ros/param.h>
#include <ros/console.h>

#include <vector>
#include <chrono>

namespace mav_active_3d_planning {

    TrajectoryEvaluator::TrajectoryEvaluator(voxblox::EsdfServer *voxblox_ptr, std::string param_ns)
            : voxblox_ptr_(voxblox_ptr),
              bounding_volume_(param_ns + "/bounding_volume"),
              cost_computer_(nullptr),
              value_computer_(nullptr),
              next_selector_(nullptr),
              p_namespace_(param_ns) {}

    bool TrajectoryEvaluator::computeGain(TrajectorySegment &traj_in) {
        // This is the main responsibility of a trajectory evaluator, no default implementation.
        ROS_ERROR("TrajectoryEvaluator::computeGain() is not implemented!");
        return false;
    }

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

    RayCaster::RayCaster(voxblox::EsdfServer *voxblox_ptr, std::string param_ns) : voxblox_ptr_(voxblox_ptr) {
        // Cache constants from voxblox
        c_voxel_size_ = voxblox_ptr_->getEsdfMapPtr()->voxel_size();
        c_block_size_ = voxblox_ptr_->getEsdfMapPtr()->block_size();

        // get params from ros
        std::string camera_params_ns;
        ros::param::param<std::string>(param_ns+"/camera_params_ns", camera_params_ns, param_ns);
        ros::param::param<double>(param_ns+"/ray_length", p_ray_length_, 5.0);
        ros::param::param<double>(param_ns+"/ray_step", p_ray_step_, static_cast<double>(c_voxel_size_));

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
} // namepsace mav_active_3d_planning