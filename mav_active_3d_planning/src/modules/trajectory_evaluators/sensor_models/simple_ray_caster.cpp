#include "mav_active_3d_planning/modules/trajectory_evaluators/sensor_models/simple_ray_caster.h"

#include <vector>
#include <algorithm>
#include <cmath>

namespace mav_active_3d_planning {
    namespace sensor_models {

        ModuleFactory::Registration <SimpleRayCaster> SimpleRayCaster::registration("SimpleRayCaster");

        void SimpleRayCaster::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<double>(param_map, "ray_step", &p_ray_step_, (double) c_voxel_size_);
            setParam<double>(param_map, "downsampling_factor", &p_downsampling_factor_, 1.0);

            // setup parent
            CameraModel::setupFromParamMap(param_map);

            // Downsample to voxel size resolution at max range
            c_res_x_ = std::min(
                    (int) ceil(p_ray_length_ * c_field_of_view_x_ / ((double) c_voxel_size_ * p_downsampling_factor_)),
                    p_resolution_x_);
            c_res_y_ = std::min(
                    (int) ceil(p_ray_length_ * c_field_of_view_y_ / ((double) c_voxel_size_ * p_downsampling_factor_)),
                    p_resolution_y_);
        }

        bool SimpleRayCaster::getVisibleVoxels(std::vector <Eigen::Vector3d> *result, const Eigen::Vector3d &position,
                                               const Eigen::Quaterniond &orientation) {
            // Naive ray-casting
            Eigen::Vector3d camera_direction;
            Eigen::Vector3d direction;
            Eigen::Vector3d current_position;
            for (int i = 0; i < c_res_x_; ++i) {
                for (int j = 0; j < c_res_y_; ++j) {
                    CameraModel::getDirectionVector(&camera_direction, (double) i / ((double) c_res_x_ - 1.0),
                                                    (double) j / ((double) c_res_y_ - 1.0));
                    direction = orientation * camera_direction;
                    double distance = 0.0;
                    while (distance < p_ray_length_) {
                        current_position = position + distance * direction;
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

    } // namespace sensor_models
} // namepsace mav_active_3d_planning