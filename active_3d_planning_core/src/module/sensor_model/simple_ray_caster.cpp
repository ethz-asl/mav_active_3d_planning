#include "active_3d_planning_core/module/sensor_model/simple_ray_caster.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace active_3d_planning {
namespace sensor_model {

ModuleFactoryRegistry::Registration<SimpleRayCaster>
    SimpleRayCaster::registration("SimpleRayCaster");

SimpleRayCaster::SimpleRayCaster(PlannerI& planner) : CameraModel(planner) {}

void SimpleRayCaster::setupFromParamMap(Module::ParamMap* param_map) {
  CameraModel::setupFromParamMap(param_map);
  setParam<double>(param_map, "ray_step", &p_ray_step_, map_->getVoxelSize());
  setParam<double>(param_map, "downsampling_factor", &p_downsampling_factor_,
                   1.0);

  // Downsample to voxel size resolution at max range
  c_res_x_ = std::min(
      static_cast<int>(ceil(p_ray_length_ * c_field_of_view_x_ /
                            (map_->getVoxelSize() * p_downsampling_factor_))),
      p_resolution_x_);
  c_res_y_ = std::min(
      static_cast<int>(ceil(p_ray_length_ * c_field_of_view_y_ /
                            (map_->getVoxelSize() * p_downsampling_factor_))),
      p_resolution_y_);
}

bool SimpleRayCaster::getVisibleVoxels(std::vector<Eigen::Vector3d>* result,
                                       const Eigen::Vector3d& position,
                                       const Eigen::Quaterniond& orientation) {
  // Naive ray-casting
  Eigen::Vector3d camera_direction;
  Eigen::Vector3d direction;
  Eigen::Vector3d current_position;
  Eigen::Vector3d voxel_center;
  for (int i = 0; i < c_res_x_; ++i) {
    for (int j = 0; j < c_res_y_; ++j) {
      CameraModel::getDirectionVector(
          &camera_direction,
          static_cast<double>(i) / (static_cast<double>(c_res_x_) - 1.0),
          static_cast<double>(j) / (static_cast<double>(c_res_y_) - 1.0));
      direction = orientation * camera_direction;
      double distance = 0.0;
      while (distance < p_ray_length_) {
        current_position = position + distance * direction;
        distance += p_ray_step_;

        // Check voxel occupied
        if (map_->getVoxelState(current_position) ==
            map::OccupancyMap::OCCUPIED) {
          break;
        }

        // Add point (duplicates are handled in
        // CameraModel::getVisibleVoxelsFromTrajectory)
        map_->getVoxelCenter(&voxel_center, current_position);
        result->push_back(voxel_center);
      }
    }
  }
  return true;
}

}  // namespace sensor_model
}  // namespace active_3d_planning
