#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_CONTINUOUS_YAW_PLANNING_EVALUATOR_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_CONTINUOUS_YAW_PLANNING_EVALUATOR_H_

#include "active_3d_planning_core/module/trajectory_evaluator/yaw_planning_evaluator.h"

namespace active_3d_planning {
namespace evaluator_updater {
class YawPlanningUpdater;
}
namespace trajectory_evaluator {

// Split full yaw into sections and find best orientation by conbining sections.
// From this paper: https://ieeexplore.ieee.org/abstract/document/8594502
// Samples yaws uniformly and assigns the same yaw to all trajectory points.
// Make sure the horizontal FOV of the sensor model matches the spacing of the
// evaluator
class ContinuousYawPlanningEvaluator : public YawPlanningEvaluator {
 public:
  explicit ContinuousYawPlanningEvaluator(PlannerI& planner);  // NOLINT

  // Override virtual functions
  bool computeGain(TrajectorySegment* traj_in) override;

  void visualizeTrajectoryValue(VisualizationMarkers* markers,
                                const TrajectorySegment& trajectory) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  friend class evaluator_updater::YawPlanningUpdater;

  // factory access

  static ModuleFactoryRegistry::Registration<ContinuousYawPlanningEvaluator>
      registration;

  // params
  bool p_visualize_followup_;  // true: also visualize the gain of the best
  // orientation
  int p_n_sections_fov_;   // Number of sections visible, i.e. fov/section_width

  // methods
  double sampleYaw(double original_yaw, int sample) override;

  void setTrajectoryYaw(TrajectorySegment* segment, double start_yaw,
                        double target_yaw) override;

  void setBestYaw(TrajectorySegment* segment) override;
};

}  // namespace trajectory_evaluator
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_CONTINUOUS_YAW_PLANNING_EVALUATOR_H_
