#include "active_3d_planning_core/module/sensor_model/iterative_ray_caster_lidar.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

namespace active_3d_planning {
namespace sensor_model {

ModuleFactoryRegistry::Registration<IterativeRayCasterLidar>
    IterativeRayCasterLidar::registration("IterativeRayCasterLidar");

IterativeRayCasterLidar::IterativeRayCasterLidar(PlannerI& planner)
    : LidarModel(planner) {}

void IterativeRayCasterLidar::setupFromParamMap(Module::ParamMap* param_map) {
  LidarModel::setupFromParamMap(param_map);
  setParam<double>(param_map, "ray_step", &p_ray_step_, map_->getVoxelSize());
  setParam<double>(param_map, "downsampling_factor", &p_downsampling_factor_,
                   1.0);

  // Downsample to voxel size resolution at max range
  c_res_x_ = std::min(static_cast<int>(std::ceil(
                          p_ray_length_ * p_fov_x_ /
                          (map_->getVoxelSize() * p_downsampling_factor_))),
                      p_resolution_x_);
  c_res_y_ = std::min(static_cast<int>(std::ceil(
                          p_ray_length_ * p_fov_y_ /
                          (map_->getVoxelSize() * p_downsampling_factor_))),
                      p_resolution_y_);

  // Determine number of splits + split distances
  c_n_sections_ =
      std::floor(static_cast<double>(std::log2(std::min(c_res_x_, c_res_y_))));
  c_split_widths_.push_back(0);
  for (int i = 0; i < c_n_sections_; ++i) {
    c_split_widths_.push_back(std::pow(2, i));
    c_split_distances_.push_back(p_ray_length_ /
                                 std::pow(2.0, static_cast<double>(i)));
  }
  c_split_distances_.push_back(0.0);
  std::reverse(c_split_distances_.begin(), c_split_distances_.end());
  std::reverse(c_split_widths_.begin(), c_split_widths_.end());
}

bool IterativeRayCasterLidar::getVisibleVoxels(
    std::vector<Eigen::Vector3d>* result, const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation) {
  // Setup ray table (contains at which segment to start, -1 if occluded
  ray_table_ = Eigen::ArrayXXi::Zero(c_res_x_, c_res_y_);

  // Ray-casting
  Eigen::Vector3d camera_direction;
  Eigen::Vector3d direction;
  Eigen::Vector3d current_position;
  Eigen::Vector3d voxel_center;
  double distance;
  bool cast_ray;
  double map_distance;
  for (int i = 0; i < c_res_x_; ++i) {
    for (int j = 0; j < c_res_y_; ++j) {
      int current_segment = ray_table_(i, j);  // get ray starting segment
      if (current_segment < 0) {
        continue;  // already occluded ray
      }
      LidarModel::getDirectionVector(
          &camera_direction,
          static_cast<double>(i) / (static_cast<double>(c_res_x_) - 1.0),
          static_cast<double>(j) / (static_cast<double>(c_res_y_) - 1.0));
      direction = orientation * camera_direction;
      distance = c_split_distances_[current_segment];
      cast_ray = true;
      while (cast_ray) {
        // iterate through all splits (segments)
        while (distance < c_split_distances_[current_segment + 1]) {
          current_position = position + distance * direction;
          distance += p_ray_step_;

          // Add point (duplicates are handled in
          // CameraModel::getVisibleVoxelsFromTrajectory)
          map_->getVoxelCenter(&voxel_center, current_position);
          result->push_back(voxel_center);

          // Check voxel occupied
          if (map_->getVoxelState(current_position) ==
              map::OccupancyMap::OCCUPIED) {
            // Occlusion, mark neighboring rays as occluded
            markNeighboringRays(i, j, current_segment, -1);
            cast_ray = false;
            break;
          }
        }
        if (cast_ray) {
          current_segment++;
          if (current_segment >= c_n_sections_) {
            cast_ray = false;  // done
          } else {
            // update ray starts of neighboring rays
            markNeighboringRays(i, j, current_segment - 1, current_segment);
          }
        }
      }
    }
  }
  return true;
}

void IterativeRayCasterLidar::markNeighboringRays(int x, int y, int segment,
                                                  int value) {
  // Set all nearby (towards bottom right) ray starts, depending on the segment
  // depth, to a value.
  for (int i = x; i < std::min(c_res_x_, x + c_split_widths_[segment]); ++i) {
    for (int j = y; j < std::min(c_res_y_, y + c_split_widths_[segment]); ++j) {
      ray_table_(i, j) = value;
    }
  }
}

}  // namespace sensor_model
}  // namespace active_3d_planning
