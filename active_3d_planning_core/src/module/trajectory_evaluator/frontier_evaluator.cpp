#include "active_3d_planning_core/module/trajectory_evaluator/frontier_evaluator.h"

#include <algorithm>

namespace active_3d_planning {
namespace trajectory_evaluator {

// Register to factory
ModuleFactoryRegistry::Registration<FrontierEvaluator>
    FrontierEvaluator::registration("FrontierEvaluator");

FrontierEvaluator::FrontierEvaluator(PlannerI& planner)
    : SimulatedSensorEvaluator(planner) {}

void FrontierEvaluator::setupFromParamMap(Module::ParamMap* param_map) {
  SimulatedSensorEvaluator::setupFromParamMap(param_map);
  setParam<bool>(param_map, "accurate_frontiers", &p_accurate_frontiers_,
                 false);
  setParam<bool>(param_map, "surface_frontiers", &p_surface_frontiers_, true);
  setParam<double>(param_map, "checking_distance", &p_checking_distance_, 1.0);

  // setup map
  map_ = dynamic_cast<map::OccupancyMap*>(&(planner_.getMap()));
  if (!map_) {
    planner_.printError(
        "'FrontierEvaluator' requires a map of type 'OccupancyMap'!");
  }

  // initialize neighbor offsets
  c_voxel_size_ = map_->getVoxelSize();
  auto vs = c_voxel_size_ * p_checking_distance_;
  if (!p_accurate_frontiers_) {
    c_neighbor_voxels_[0] = Eigen::Vector3d(vs, 0, 0);
    c_neighbor_voxels_[1] = Eigen::Vector3d(-vs, 0, 0);
    c_neighbor_voxels_[2] = Eigen::Vector3d(0, vs, 0);
    c_neighbor_voxels_[3] = Eigen::Vector3d(0, -vs, 0);
    c_neighbor_voxels_[4] = Eigen::Vector3d(0, 0, vs);
    c_neighbor_voxels_[5] = Eigen::Vector3d(0, 0, -vs);
  } else {
    c_neighbor_voxels_[0] = Eigen::Vector3d(vs, 0, 0);
    c_neighbor_voxels_[1] = Eigen::Vector3d(vs, vs, 0);
    c_neighbor_voxels_[2] = Eigen::Vector3d(vs, -vs, 0);
    c_neighbor_voxels_[3] = Eigen::Vector3d(vs, 0, vs);
    c_neighbor_voxels_[4] = Eigen::Vector3d(vs, vs, vs);
    c_neighbor_voxels_[5] = Eigen::Vector3d(vs, -vs, vs);
    c_neighbor_voxels_[6] = Eigen::Vector3d(vs, 0, -vs);
    c_neighbor_voxels_[7] = Eigen::Vector3d(vs, vs, -vs);
    c_neighbor_voxels_[8] = Eigen::Vector3d(vs, -vs, -vs);
    c_neighbor_voxels_[9] = Eigen::Vector3d(0, vs, 0);
    c_neighbor_voxels_[10] = Eigen::Vector3d(0, -vs, 0);
    c_neighbor_voxels_[11] = Eigen::Vector3d(0, 0, vs);
    c_neighbor_voxels_[12] = Eigen::Vector3d(0, vs, vs);
    c_neighbor_voxels_[13] = Eigen::Vector3d(0, -vs, vs);
    c_neighbor_voxels_[14] = Eigen::Vector3d(0, 0, -vs);
    c_neighbor_voxels_[15] = Eigen::Vector3d(0, vs, -vs);
    c_neighbor_voxels_[16] = Eigen::Vector3d(0, -vs, -vs);
    c_neighbor_voxels_[17] = Eigen::Vector3d(-vs, 0, 0);
    c_neighbor_voxels_[18] = Eigen::Vector3d(-vs, vs, 0);
    c_neighbor_voxels_[19] = Eigen::Vector3d(-vs, -vs, 0);
    c_neighbor_voxels_[20] = Eigen::Vector3d(-vs, 0, vs);
    c_neighbor_voxels_[21] = Eigen::Vector3d(-vs, vs, vs);
    c_neighbor_voxels_[22] = Eigen::Vector3d(-vs, -vs, vs);
    c_neighbor_voxels_[23] = Eigen::Vector3d(-vs, 0, -vs);
    c_neighbor_voxels_[24] = Eigen::Vector3d(-vs, vs, -vs);
    c_neighbor_voxels_[25] = Eigen::Vector3d(-vs, -vs, -vs);
  }
}

bool FrontierEvaluator::isFrontierVoxel(const Eigen::Vector3d& voxel) {
  // Check all neighboring voxels
  unsigned char voxel_state;
  if (!p_accurate_frontiers_) {
    for (int i = 0; i < 6; ++i) {
      voxel_state = map_->getVoxelState(voxel + c_neighbor_voxels_[i]);
      if (voxel_state == map::OccupancyMap::UNKNOWN) {
        continue;
      }
      if (p_surface_frontiers_) {
        return voxel_state == map::OccupancyMap::OCCUPIED;
      } else {
        return true;
      }
    }
  } else {
    for (int i = 0; i < 26; ++i) {
      voxel_state = map_->getVoxelState(voxel + c_neighbor_voxels_[i]);
      if (voxel_state == map::OccupancyMap::UNKNOWN) {
        continue;
      }
      if (p_surface_frontiers_) {
        return voxel_state == map::OccupancyMap::OCCUPIED;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool FrontierEvaluator::computeGainFromVisibleVoxels(
    TrajectorySegment* traj_in) {
  traj_in->gain = 0.0;
  if (!traj_in->info) {
    return false;
  }
  SimulatedSensorInfo* info =
      reinterpret_cast<SimulatedSensorInfo*>(traj_in->info.get());

  // Remove observed voxels
  info->visible_voxels.erase(
      std::remove_if(info->visible_voxels.begin(), info->visible_voxels.end(),
                     [this](const Eigen::Vector3d& voxel) {
                       return map_->isObserved(voxel);
                     }),
      info->visible_voxels.end());

  // Check which voxels are frontier voxels
  for (int i = 0; i < info->visible_voxels.size(); ++i) {
    if (isFrontierVoxel(info->visible_voxels[i])) {
      traj_in->gain += 1.0;
    }
  }
  return true;
}

void FrontierEvaluator::visualizeTrajectoryValue(
    VisualizationMarkers* markers, const TrajectorySegment& trajectory) {
  // Default implementation displays all frontier voxels
  if (!trajectory.info) {
    return;
  }
  VisualizationMarker marker;
  marker.type = VisualizationMarker::CUBE_LIST;
  double voxel_size = map_->getVoxelSize();
  marker.scale.x() = voxel_size;
  marker.scale.y() = voxel_size;
  marker.scale.z() = voxel_size;
  marker.color.r = 1.0;
  marker.color.g = 0.8;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  // points
  SimulatedSensorInfo* info =
      reinterpret_cast<SimulatedSensorInfo*>(trajectory.info.get());
  for (int i = 0; i < info->visible_voxels.size(); ++i) {
    if (isFrontierVoxel(info->visible_voxels[i])) {
      marker.points.push_back(info->visible_voxels[i]);
    }
  }
  markers->addMarker(marker);

  if (p_visualize_sensor_view_) {
    sensor_model_->visualizeSensorView(markers, trajectory);
  }
}

}  // namespace trajectory_evaluator
}  // namespace active_3d_planning
