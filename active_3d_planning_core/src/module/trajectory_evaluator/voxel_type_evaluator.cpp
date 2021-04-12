#include "active_3d_planning_core/module/trajectory_evaluator/voxel_type_evaluator.h"

#include <algorithm>
#include <string>

#include "active_3d_planning_core/module/module_factory.h"
#include "active_3d_planning_core/planner/planner_I.h"

namespace active_3d_planning {
namespace trajectory_evaluator {

// Factory Registration
ModuleFactoryRegistry::Registration<VoxelTypeEvaluator>
    VoxelTypeEvaluator::registration("VoxelTypeEvaluator");

VoxelTypeEvaluator::VoxelTypeEvaluator(PlannerI& planner)
    : SimulatedSensorEvaluator(planner) {}

void VoxelTypeEvaluator::setupFromParamMap(Module::ParamMap* param_map) {
  // setup parent
  SimulatedSensorEvaluator::setupFromParamMap(param_map);

  // params
  setParam<double>(param_map, "gain_unknown", &p_gain_unknown_, 1.0);
  setParam<double>(param_map, "gain_occupied", &p_gain_occupied_, 0.0);
  setParam<double>(param_map, "gain_free", &p_gain_free_, 0.0);
  setParam<double>(param_map, "gain_unknown_outer", &p_gain_unknown_outer_,
                   0.0);
  setParam<double>(param_map, "gain_occupied_outer", &p_gain_occupied_outer_,
                   0.0);
  setParam<double>(param_map, "gain_free_outer", &p_gain_free_outer_, 0.0);

  // setup map
  map_ = dynamic_cast<map::OccupancyMap*>(&(planner_.getMap()));
  if (!map_) {
    planner_.printError(
        "'FrontierEvaluator' requires a map of type 'OccupancyMap'!");
  }

  // outer volume (if not specified will not be counted)
  std::string ns = (*param_map)["param_namespace"];
  std::string outer_volume_args;
  setParam<std::string>(param_map, "outer_volume_args", &outer_volume_args,
                        ns + "/outer_volume");
  outer_volume_ = planner_.getFactory().createModule<BoundingVolume>(
      outer_volume_args, planner_, verbose_modules_);

  // constants
  c_min_gain_ = std::min({p_gain_unknown_, p_gain_occupied_, p_gain_free_,
                          p_gain_unknown_outer_, p_gain_occupied_outer_,
                          p_gain_free_outer_});
  c_max_gain_ = std::max({p_gain_unknown_, p_gain_occupied_, p_gain_free_,
                          p_gain_unknown_outer_, p_gain_occupied_outer_,
                          p_gain_free_outer_});
}

bool VoxelTypeEvaluator::computeGainFromVisibleVoxels(
    TrajectorySegment* traj_in) {
  traj_in->gain = 0.0;
  if (!traj_in->info) {
    return false;
  }
  SimulatedSensorInfo* info =
      reinterpret_cast<SimulatedSensorInfo*>(traj_in->info.get());
  unsigned char voxel_state;
  for (int i = 0; i < info->visible_voxels.size(); ++i) {
    voxel_state = map_->getVoxelState(info->visible_voxels[i]);
    if (bounding_volume_->contains(info->visible_voxels[i])) {
      switch (voxel_state) {
        case map::OccupancyMap::UNKNOWN:
          traj_in->gain += p_gain_unknown_;
        case map::OccupancyMap::FREE:
          traj_in->gain += p_gain_free_;
        case map::OccupancyMap::OCCUPIED:
          traj_in->gain += p_gain_occupied_;
      }
    } else {
      switch (voxel_state) {
        case map::OccupancyMap::UNKNOWN:
          traj_in->gain += p_gain_unknown_outer_;
        case map::OccupancyMap::FREE:
          traj_in->gain += p_gain_free_outer_;
        case map::OccupancyMap::OCCUPIED:
          traj_in->gain += p_gain_occupied_outer_;
      }
    }
  }
  return true;
}

}  // namespace trajectory_evaluator
}  // namespace active_3d_planning
