#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VOXEL_WEIGHT_EVALUATOR_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VOXEL_WEIGHT_EVALUATOR_H_

#include <vector>

#include "active_3d_planning_core/map/tsdf_map.h"
#include "active_3d_planning_core/module/trajectory_evaluator/frontier_evaluator.h"

namespace active_3d_planning {
namespace trajectory_evaluator {

// VoxelWeight uses the tsdf weight of surface voxels to estimate how much they
// can still change with additional observations. Requires the TSDF map to
// published to the planner intern voxblox server. Uses frontier voxels to allow
// additional gain for exploration. Surface voxels contribute between [0, 1] per
// voxel, based on the expectd impact. Unobserved voxels contribute 1x
// new_voxel_weight per voxel.
class VoxelWeightEvaluator : public FrontierEvaluator {
 public:
  explicit VoxelWeightEvaluator(PlannerI& planner);  // NOLINT

  // Override virtual methods
  void visualizeTrajectoryValue(VisualizationMarkers* markers,
                                const TrajectorySegment& trajectory) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<VoxelWeightEvaluator> registration;

  // Override virtual methods
  bool storeTrajectoryInformation(
      TrajectorySegment* traj_in,
      const std::vector<Eigen::Vector3d>& new_voxels) override;

  bool computeGainFromVisibleVoxels(TrajectorySegment* traj_in) override;

  // map
  map::TSDFMap* map_;

  // params
  double p_min_impact_factor_;  // Minimum expected change, the gain is set at 0
  // here.
  double p_new_voxel_weight_;  // Multiply unobserved voxels by this weight to
  // balance quality/exploration
  double p_frontier_voxel_weight_;  // Multiply frontier voxels by this weight
  double
      p_ray_angle_x_;  // Angle [rad] spanned between 2 pixels in x direction,
  // i.e. fov/res
  double p_ray_angle_y_;

  // constants
  double c_voxel_size_;

  // methods
  double getVoxelValue(const Eigen::Vector3d& voxel,
                       const Eigen::Vector3d& origin);
};

}  // namespace trajectory_evaluator
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VOXEL_WEIGHT_EVALUATOR_H_
