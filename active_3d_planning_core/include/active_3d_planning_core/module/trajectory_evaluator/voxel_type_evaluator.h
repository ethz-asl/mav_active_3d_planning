#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VOXEL_TYPE_EVALUATOR_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VOXEL_TYPE_EVALUATOR_H_

#include <memory>

#include "active_3d_planning_core/map/occupancy_map.h"
#include "active_3d_planning_core/module/trajectory_evaluator/simulated_sensor_evaluator.h"

namespace active_3d_planning {
namespace trajectory_evaluator {

// VoxelType assigns a constant value to every kind of visible value
// (unobserved/free/occupied, inside/outside target bounding volume)
class VoxelTypeEvaluator : public SimulatedSensorEvaluator {
 public:
  explicit VoxelTypeEvaluator(PlannerI& planner);  // NOLINT

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<VoxelTypeEvaluator> registration;

  // members
  std::unique_ptr<BoundingVolume> outer_volume_;
  map::OccupancyMap* map_;

  // parameters
  double p_gain_unknown_;
  double p_gain_unknown_outer_;
  double p_gain_occupied_;
  double p_gain_occupied_outer_;
  double p_gain_free_;
  double p_gain_free_outer_;

  // constants
  double c_min_gain_;
  double c_max_gain_;

  // Override virtual methods
  bool computeGainFromVisibleVoxels(TrajectorySegment* traj_in) override;
};

}  // namespace trajectory_evaluator
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VOXEL_TYPE_EVALUATOR_H_
