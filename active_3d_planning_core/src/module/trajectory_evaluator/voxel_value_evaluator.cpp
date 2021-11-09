#include "active_3d_planning_core/module/trajectory_evaluator/voxel_value_evaluator.h"

#include <iomanip>
#include <vector>

#include <active_3d_planning_core/map/occupancy_map.h>
#include <active_3d_planning_core/map/voxel_value_map.h>

#include "active_3d_planning_core/map/tsdf_map.h"

namespace active_3d_planning {
namespace trajectory_evaluator {

ModuleFactoryRegistry::Registration<VoxelValueEvaluator>
    VoxelValueEvaluator::registration("VoxelValueEvaluator");

VoxelValueEvaluator::VoxelValueEvaluator(PlannerI& planner)
    : FrontierEvaluator(planner) {}

void VoxelValueEvaluator::setupFromParamMap(Module::ParamMap* param_map) {
  FrontierEvaluator::setupFromParamMap(param_map);
  setParam<double>(param_map, "frontier_voxel_value", &p_frontier_voxel_value_,
                   0.1);
  setParam<double>(param_map, "min_impact_factor", &p_min_impact_factor_, 0.0);
  setParam<double>(param_map, "new_voxel_value", &p_new_voxel_value_, 0.5);
  setParam<double>(param_map, "ray_angle_x", &p_ray_angle_x_, 0.0025);
  setParam<double>(param_map, "ray_angle_y", &p_ray_angle_y_, 0.0025);
  setParam<std::string>(param_map, "discount_factor", &p_discount_factor,
                        "weights");
  setParam<double>(param_map, "value_factor", &p_value_factor, 1.0);
  // setup map
  map_ = dynamic_cast<map::VoxelValueMap*>(&(planner_.getMap()));
  if (!map_) {
    planner_.printError(
        "'VoxelValueEvaluator' requires a map of type 'VoxelValueMap'!");
  }

  // cache voxblox constants
  c_voxel_size_ = map_->getVoxelSize();
}

bool VoxelValueEvaluator::storeTrajectoryInformation(
    TrajectorySegment* traj_in,
    const std::vector<Eigen::Vector3d>& new_voxels) {
  // Uses the default voxel info, not much gain from caching more info
  return SimulatedSensorEvaluator::storeTrajectoryInformation(traj_in,
                                                              new_voxels);
}

bool VoxelValueEvaluator::computeGainFromVisibleVoxels(
    TrajectorySegment* traj_in) {
  traj_in->gain = 0.0;
  if (!traj_in->info) {
    return false;
  }
  SimulatedSensorInfo* info =
      reinterpret_cast<SimulatedSensorInfo*>(traj_in->info.get());
  // just assume we take a single image from the last trajectory point here...
  Eigen::Vector3d origin = traj_in->trajectory.back().position_W;
  for (int i = 0; i < info->visible_voxels.size(); ++i) {
    traj_in->gain += getVoxelValue(info->visible_voxels[i], origin);
  }
  return true;
}

double VoxelValueEvaluator::getVoxelValue(const Eigen::Vector3d& voxel,
                                          const Eigen::Vector3d& origin) {
  unsigned char voxel_state = map_->getValueVoxelState(voxel);

  if (voxel_state == map::TSDFMap::OCCUPIED) {
    // OCCUPIED -> Get value from Map
    // Surface voxel
    double discount_factor = 1;
    if (p_discount_factor == "linear") {
      // factor = 1/(N+1)
      discount_factor = 1.0 / (1.0 + map_->getVoxelObservedCount(voxel));
    } else if (p_discount_factor == "weights") {
      // Factor = new_weight / (new_weight + map_weight).
      // Also has threshold p_min_impact_factor_ if required.
      double min_distance = 1;
      double z = std::max(min_distance, (voxel - origin).norm());
      double spanned_angle = 2.0 * atan2(c_voxel_size_, z * 2.0);
      double new_weight = std::pow(spanned_angle, 2.0) /
                          (p_ray_angle_x_ * p_ray_angle_y_) / std::pow(z, 2.0);

      double gain = new_weight / (new_weight + map_->getVoxelWeight(voxel));
      if (gain > p_min_impact_factor_) {
        (gain - p_min_impact_factor_) / (1 - p_min_impact_factor_);
      } else
        discount_factor = 0;
    }
    return p_value_factor * discount_factor * map_->getVoxelValue(voxel);
  } else if (voxel_state == map::TSDFMap::UNKNOWN) {
    // Unobserved voxels
    return p_new_voxel_value_;
  }
  return 0;
}

void VoxelValueEvaluator::visualizeTrajectoryValue(
    VisualizationMarkers* markers, const TrajectorySegment& trajectory) {
  // Display all voxels that contribute to the gain. max_impact-min_impact as
  // green-red, frontier voxels purple, unknwon voxels teal
  if (!trajectory.info) {
    return;
  }

  VisualizationMarker marker;
  marker.type = VisualizationMarker::CUBE_LIST;
  marker.scale.x() = c_voxel_size_;
  marker.scale.y() = c_voxel_size_;
  marker.scale.z() = c_voxel_size_;

  // points
  double value;
  Eigen::Vector3d origin = trajectory.trajectory.back().position_W;
  SimulatedSensorInfo* info =
      reinterpret_cast<SimulatedSensorInfo*>(trajectory.info.get());
  for (int i = 0; i < info->visible_voxels.size(); ++i) {
    value = getVoxelValue(info->visible_voxels[i], origin);

    if (value > 0.0) {
      marker.points.push_back(info->visible_voxels[i]);
      Color color;
      if (value == p_new_voxel_value_) {
        color.r = 0;
        color.g = 1;
        color.b = 0;
        color.a = 0.3;
      } else {
        double val = value / 0.1;
        if (val > 0.5) {
          if (val > 1) {
            val = 1.0f;
          }
          val = val - 0.5;
          color.r = std::min(val * 2.f, 1.0);
          color.g = std::max(1 - val * 2.f, 0.0);
          color.a = 0.3;
          color.b = 0;
        } else {
          color.g = 0;
          color.r = std::min(val * 2.f, 1.0);
          color.b = std::max(1 - val * 2.f, 0.0);
          color.a = 0.3;
        }
      }
      marker.colors.push_back(color);
    }
  }
  markers->addMarker(marker);

  if (p_visualize_sensor_view_) {
    sensor_model_->visualizeSensorView(markers, trajectory);
  }
}

}  // namespace trajectory_evaluator
}  // namespace active_3d_planning
