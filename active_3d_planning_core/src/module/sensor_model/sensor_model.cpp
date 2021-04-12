#include "active_3d_planning_core/module/sensor_model/sensor_model.h"

#include "active_3d_planning_core/planner/planner_I.h"

namespace active_3d_planning {

SensorModel::SensorModel(PlannerI& planner) : Module(planner) {}

void SensorModel::setupFromParamMap(Module::ParamMap* param_map) {
  // setup map
  map_ = dynamic_cast<map::OccupancyMap*>(&(planner_.getMap()));
  if (!map_) {
    planner_.printError("'SensorModel' requires a map of type 'OccupancyMap'!");
  }

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

}  // namespace active_3d_planning
