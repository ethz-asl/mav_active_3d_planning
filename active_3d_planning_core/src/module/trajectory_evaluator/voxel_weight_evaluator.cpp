#include "active_3d_planning/module/trajectory_evaluator/voxel_weight_evaluator.h"

#include <voxblox/core/tsdf_map.h>

namespace active_3d_planning {
namespace trajectory_evaluator {

ModuleFactoryRegistry::Registration<VoxelWeightEvaluator>
    VoxelWeightEvaluator::registration("VoxelWeightEvaluator");

VoxelWeightEvaluator::VoxelWeightEvaluator(PlannerI &planner)
    : FrontierEvaluator(planner), voxblox_tsdf_(planner.getTsdfMap()) {}

void VoxelWeightEvaluator::setupFromParamMap(Module::ParamMap *param_map) {
  FrontierEvaluator::setupFromParamMap(param_map);
  setParam<double>(param_map, "frontier_voxel_weight",
                   &p_frontier_voxel_weight_, 1.0);
  setParam<double>(param_map, "min_impact_factor", &p_min_impact_factor_, 0.0);
  setParam<double>(param_map, "new_voxel_weight", &p_new_voxel_weight_, 0.01);
  setParam<double>(param_map, "ray_angle_x", &p_ray_angle_x_, 0.0025);
  setParam<double>(param_map, "ray_angle_y", &p_ray_angle_y_, 0.0025);

  // cache voxblox constants
  c_voxel_size_ = (double)voxblox_.voxel_size();
}

bool VoxelWeightEvaluator::storeTrajectoryInformation(
    TrajectorySegment *traj_in,
    const std::vector<Eigen::Vector3d> &new_voxels) {
  // Uses the default voxel info, not much gain from caching more info
  return SimulatedSensorEvaluator::storeTrajectoryInformation(traj_in,
                                                              new_voxels);
}

bool VoxelWeightEvaluator::computeGainFromVisibleVoxels(
    TrajectorySegment *traj_in) {
  traj_in->gain = 0.0;
  if (!traj_in->info) {
    return false;
  }
  SimulatedSensorInfo *info =
      reinterpret_cast<SimulatedSensorInfo *>(traj_in->info.get());

  // just assume we take a single image from the last trajectory point here...
  Eigen::Vector3d origin = traj_in->trajectory.back().position_W;
  for (int i = 0; i < info->visible_voxels.size(); ++i) {
    traj_in->gain += getVoxelValue(info->visible_voxels[i], origin);
  }
  return true;
}

double VoxelWeightEvaluator::getVoxelValue(const Eigen::Vector3d &voxel,
                                           const Eigen::Vector3d &origin) {
  double distance;
  if (voxblox_.getDistanceAtPosition(voxel, &distance)) {
    // voxel is observed
    if (distance >= 0.0 && distance < c_voxel_size_) {
      // Surface voxel
      voxblox::Point point(voxel.x(), voxel.y(), voxel.z());
      voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
          voxblox_.getTsdfMapPtr()->getTsdfLayerPtr()->getBlockPtrByCoordinates(
              point);
      if (block) {
        voxblox::TsdfVoxel *tsdf_voxel = block->getVoxelPtrByCoordinates(point);
        if (tsdf_voxel) {
          // TSDF voxel is also observed, compute the estimated impact (weight
          // ratio)
          double z = (voxel - origin).norm();
          double spanned_angle = 2.0 * atan2(c_voxel_size_, z * 2.0);
          double new_weight = std::pow(spanned_angle, 2.0) /
                              (p_ray_angle_x_ * p_ray_angle_y_) /
                              std::pow(z, 2.0);
          double gain =
              new_weight / (new_weight + (double)(tsdf_voxel->weight));
          if (gain > p_min_impact_factor_) {
            return gain;
          } else {
            return 0.0;
          }
        }
      }
    } else {
      // non-surface voxel
      return 0.0;
    }
  }
  // Unobserved voxels
  if (p_frontier_voxel_weight_ > 0.0) {
    if (isFrontierVoxel(voxel)) {
      return p_frontier_voxel_weight_;
    }
  }
  return p_new_voxel_weight_;
}

void VoxelWeightEvaluator::visualizeTrajectoryValue(
    VisualizerI &visualizer, const TrajectorySegment &trajectory) {
  // Display all voxels that contribute to the gain. max_impact-min_impact as
  // green-red, unknwon voxels purple
  if (!trajectory.info) {
    return;
  }
  VisualizationMarker marker;
  marker.type = VisualizationMarker::CUBE_LIST;
  marker.scale.x = c_voxel_size_;
  marker.scale.y = c_voxel_size_;
  marker.scale.z = c_voxel_size_;

  // points
  double value;
  Eigen::Vector3d origin = trajectory.trajectory.back().position_W;
  SimulatedSensorInfo *info =
      reinterpret_cast<SimulatedSensorInfo *>(trajectory.info.get());
  for (int i = 0; i < info->visible_voxels.size(); ++i) {
    value = getVoxelValue(info->visible_voxels[i], origin);
    if (value > 0.0) {
      marker.points.push_back(info->visible_voxels[i]);
      Color color;
      if (value == p_frontier_voxel_weight_) {
        color.r = 0.6;
        color.g = 0.4;
        color.b = 1.0;
        color.a = 0.5;
      } else if (value == p_new_voxel_weight_) {
        color.r = 0.0;
        color.g = 0.75;
        color.b = 1.0;
        color.a = 0.1;
      } else {
        double frac =
            (value - p_min_impact_factor_) / (1.0 - p_min_impact_factor_);
        color.r = std::min((0.5 - frac) * 2.0 + 1.0, 1.0);
        color.g = std::min((frac - 0.5) * 2.0 + 1.0, 1.0);
        color.b = 0.0;
        color.a = 0.5;
      }
      marker.colors.push_back(color);
    }
  }
  visualizer.addMarker(marker);

  if (p_visualize_sensor_view_) {
    sensor_model_->visualizeSensorView(visualizer, trajectory);
  }
}

} // namespace trajectory_evaluator
} // namespace active_3d_planning
