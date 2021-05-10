#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VOXEL_VALUE_EVALUATOR_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VOXEL_VALUE_EVALUATOR_H_

#include <vector>

#include <active_3d_planning_core/map/voxel_value_map.h>

#include "active_3d_planning_core/module/trajectory_evaluator/frontier_evaluator.h"

namespace active_3d_planning {
namespace trajectory_evaluator {

// VoxelValue uses a value stored in the tsdf of surface voxels as gain to
// evaluate a given trajectory.
// Requires the map to be an instance of VoxelValueMap.
// Uses frontier voxels to allow additional gain for exploration.
class VoxelValueEvaluator : public FrontierEvaluator {
 public:
  explicit VoxelValueEvaluator(PlannerI& planner);  // NOLINT

  // Override virtual methods
  void visualizeTrajectoryValue(VisualizationMarkers* markers,
                                const TrajectorySegment& trajectory) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<VoxelValueEvaluator> registration;

  // Override virtual methods
  bool storeTrajectoryInformation(
      TrajectorySegment* traj_in,
      const std::vector<Eigen::Vector3d>& new_voxels) override;

  bool computeGainFromVisibleVoxels(TrajectorySegment* traj_in) override;

  // map
  map::VoxelValueMap* map_;

  // params
  double p_min_impact_factor_;  // Minimum expected change, the gain is set at 0
  // here.
  double p_new_voxel_value_;       // Default value for unobserved voxels
  double p_frontier_voxel_value_;  // Default value for frontier voxels
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
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VOXEL_VALUE_EVALUATOR_H_
