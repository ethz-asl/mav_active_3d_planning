#include "active_3d_planning_core/module/trajectory_evaluator/simulated_sensor_evaluator.h"

#include <algorithm>
#include <string>
#include <vector>

#include "active_3d_planning_core/planner/planner_I.h"

namespace active_3d_planning {
namespace trajectory_evaluator {

// SimulatedSensorEvaluator (base class)
SimulatedSensorEvaluator::SimulatedSensorEvaluator(PlannerI& planner)
    : TrajectoryEvaluator(planner) {}

void SimulatedSensorEvaluator::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<bool>(param_map, "clear_from_parents", &p_clear_from_parents_,
                 false);
  setParam<bool>(param_map, "visualize_sensor_view", &p_visualize_sensor_view_,
                 false);

  // setup map
  map_ = dynamic_cast<map::OccupancyMap*>(&(planner_.getMap()));
  if (!map_) {
    planner_.printError(
        "'SimulatedSensorEvaluator' requires a map of type 'OccupancyMap'!");
  }

  // Register link for simulated sensor udpaters
  planner_.getFactory().registerLinkableModule("SimulatedSensorEvaluator",
                                               this);

  // Create sensor model
  std::string args;  // default args extends the parent namespace
  std::string param_ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "sensor_model_args", &args,
                        param_ns + "/sensor_model");
  sensor_model_ = planner_.getFactory().createModule<SensorModel>(
      args, planner_, verbose_modules_);

  // setup parent
  TrajectoryEvaluator::setupFromParamMap(param_map);
}

bool SimulatedSensorEvaluator::computeGain(TrajectorySegment* traj_in) {
  std::vector<Eigen::Vector3d> new_voxels;
  sensor_model_->getVisibleVoxelsFromTrajectory(&new_voxels, *traj_in);

  // Check for interesting bounding box
  if (bounding_volume_->is_setup) {
    new_voxels.erase(
        std::remove_if(new_voxels.begin(), new_voxels.end(),
                       [this](const Eigen::Vector3d& voxel) {
                         return (!bounding_volume_->contains(voxel));
                       }),
        new_voxels.end());
  }

  // Remove voxels previously seen by parent (this is quite expensive)
  if (p_clear_from_parents_) {
    TrajectorySegment* previous = traj_in->parent;
    while (previous) {
      std::vector<Eigen::Vector3d> old_voxels =
          reinterpret_cast<SimulatedSensorInfo*>(previous->info.get())
              ->visible_voxels;
      new_voxels.erase(
          std::remove_if(new_voxels.begin(), new_voxels.end(),
                         [&old_voxels](const Eigen::Vector3d& voxel) {
                           auto it = std::find(old_voxels.begin(),
                                               old_voxels.end(), voxel);
                           return (it != old_voxels.end());
                         }),
          new_voxels.end());
      previous = previous->parent;
    }
  }

  // Compute gain and store trajectory information
  storeTrajectoryInformation(traj_in, new_voxels);
  computeGainFromVisibleVoxels(traj_in);
  return true;
}

bool SimulatedSensorEvaluator::storeTrajectoryInformation(
    TrajectorySegment* traj_in,
    const std::vector<Eigen::Vector3d>& new_voxels) {
  SimulatedSensorInfo* new_info = new SimulatedSensorInfo();
  new_info->visible_voxels.assign(new_voxels.begin(), new_voxels.end());
  traj_in->info.reset(new_info);
  return true;
}

// Base Visualization: just display all visible voxels
void SimulatedSensorEvaluator::visualizeTrajectoryValue(
    VisualizationMarkers* markers, const TrajectorySegment& trajectory) {
  if (!trajectory.info) {
    return;
  }
  // Default implementation displays all visible voxels
  VisualizationMarker marker;
  marker.type = VisualizationMarker::CUBE_LIST;
  double voxel_size = map_->getVoxelSize();
  marker.scale.x() = voxel_size;
  marker.scale.y() = voxel_size;
  marker.scale.z() = voxel_size;
  marker.color.r = 1.0;
  marker.color.g = 0.8;
  marker.color.b = 0.0;
  marker.color.a = 0.4;

  // points
  SimulatedSensorInfo* info =
      reinterpret_cast<SimulatedSensorInfo*>(trajectory.info.get());
  for (int i = 0; i < info->visible_voxels.size(); ++i) {
    marker.points.push_back(info->visible_voxels[i]);
  }
  markers->addMarker(marker);

  if (p_visualize_sensor_view_) {
    sensor_model_->visualizeSensorView(markers, trajectory);
  }
}

}  // namespace trajectory_evaluator
}  // namespace active_3d_planning
