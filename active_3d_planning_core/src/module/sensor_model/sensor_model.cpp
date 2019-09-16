#include "active_3d_planning/module/sensor_model/sensor_model.h"
#include "active_3d_planning/planner/planner_I.h"

#include <voxblox/core/esdf_map.h>

namespace active_3d_planning {

SensorModel::SensorModel(PlannerI &planner)
    : Module(planner), voxblox_(planner_.getMap()) {}

// SensorModel
void SensorModel::setupFromParamMap(Module::ParamMap *param_map) {
  // Cache constants from voxblox
  c_voxel_size_ = voxblox_.voxel_size();
  c_block_size_ = voxblox_.block_size();

  // setup mounting transform
  double tx, ty, tz, rx, ry, rz, rw;
  setParam<double>(param_map, "mounting_translation_x", &tx, 0.0);
  setParam<double>(param_map, "mounting_translation_y", &ty, 0.0);
  setParam<double>(param_map, "mounting_translation_z", &tz, 0.0);
  setParam<double>(param_map, "mounting_rotation_x", &rx, 0.0);
  setParam<double>(param_map, "mounting_rotation_y", &ry, 0.0);
  setParam<double>(param_map, "mounting_rotation_z", &rz, 0.0);
  setParam<double>(param_map, "mounting_rotation_w", &rw, 1.0);
  mounting_translation_ = Eigen::Vector3d(tx, ty, tz);
  mounting_rotation_ = Eigen::Quaterniond(rw, rx, ry, rz).normalized();
}

bool SensorModel::getVoxelCenter(Eigen::Vector3d *point) {
  // Move the input point to the voxel center of its voxel
  voxblox::BlockIndex block_id =
      voxblox_.getEsdfLayerPtr()->computeBlockIndexFromCoordinates(
          point->cast<voxblox::FloatingPoint>());
  Eigen::Vector3d center_point =
      voxblox::getOriginPointFromGridIndex(block_id, c_block_size_)
          .cast<double>();
  voxblox::VoxelIndex voxel_id =
      voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
          (*point - center_point).cast<voxblox::FloatingPoint>(),
          1.0 / c_voxel_size_);
  center_point += voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size_)
                      .cast<double>();
  *point = center_point;
  return true;
}

} // namespace active_3d_planning