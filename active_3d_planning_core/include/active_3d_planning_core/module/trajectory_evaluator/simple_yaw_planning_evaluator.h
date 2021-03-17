#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_SIMPLE_YAW_PLANNING_EVALUATOR_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_SIMPLE_YAW_PLANNING_EVALUATOR_H_

#include "active_3d_planning_core/module/trajectory_evaluator/yaw_planning_evaluator.h"

namespace active_3d_planning {
namespace evaluator_updater {
class YawPlanningUpdater;
}
namespace trajectory_evaluator {

// Simple evaluator. Samples yaws uniformly and assigns the same yaw to all
// trajectory points.
class SimpleYawPlanningEvaluator : public YawPlanningEvaluator {
 public:
  explicit SimpleYawPlanningEvaluator(PlannerI& planner);  // NOLINT

  // Override virtual functions
  void visualizeTrajectoryValue(VisualizationMarkers* markers,
                                const TrajectorySegment& trajectory) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  friend class evaluator_updater::YawPlanningUpdater;

  static ModuleFactoryRegistry::Registration<SimpleYawPlanningEvaluator>
      registration;

  // params
  bool p_visualize_followup_;  // true: also visualize the gain of the best
  // orientation

  // methods
  double sampleYaw(double original_yaw, int sample) override;

  void setTrajectoryYaw(TrajectorySegment* segment, double start_yaw,
                        double target_yaw) override;
};

}  // namespace trajectory_evaluator
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_SIMPLE_YAW_PLANNING_EVALUATOR_H_
