#include "active_3d_planning_core/data/bounding_volume.h"

namespace active_3d_planning {

// BoundingVolume
ModuleFactoryRegistry::Registration<BoundingVolume>
    BoundingVolume::registration("BoundingVolume");

BoundingVolume::BoundingVolume(PlannerI& planner)
    : Module(planner), is_setup(false) {}

void BoundingVolume::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<double>(param_map, "x_min", &x_min, 0.0);
  setParam<double>(param_map, "x_max", &x_max, 0.0);
  setParam<double>(param_map, "y_min", &y_min, 0.0);
  setParam<double>(param_map, "y_max", &y_max, 0.0);
  setParam<double>(param_map, "z_min", &z_min, 0.0);
  setParam<double>(param_map, "z_max", &z_max, 0.0);
  setParam<double>(param_map, "rotation", &rotation, 0.0);

  rotation_quat = Eigen::AngleAxis<double>(rotation * M_PI / -180.0,
                                           Eigen::Vector3d::UnitZ());

  // Check params span a valid volume (This is not a compulsory check to allow
  // the unspecified (default) case, where the volume is not checked)
  if (x_max > x_min && y_max > y_min && z_max > z_min) {
    is_setup = true;
  } else {
    is_setup = false;
  }
}

bool BoundingVolume::contains(const Eigen::Vector3d& point) {
  if (!is_setup) {
    return true;
  }  // Uninitialized boxes always return true, so can check them by default
  Eigen::Vector3d rotated_point = rotation_quat * point;
  if (rotated_point.x() < x_min) {
    return false;
  }
  if (rotated_point.x() > x_max) {
    return false;
  }
  if (rotated_point.y() < y_min) {
    return false;
  }
  if (rotated_point.y() > y_max) {
    return false;
  }
  if (rotated_point.z() < z_min) {
    return false;
  }
  if (rotated_point.z() > z_max) {
    return false;
  }
  return true;
}

}  // namespace active_3d_planning
