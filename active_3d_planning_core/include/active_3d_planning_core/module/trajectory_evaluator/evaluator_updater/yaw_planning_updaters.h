#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_YAW_PLANNING_UPDATERS_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_YAW_PLANNING_UPDATERS_H_

#include <memory>

#include "active_3d_planning_core/module/trajectory_evaluator/yaw_planning_evaluator.h"

namespace active_3d_planning {
namespace evaluator_updater {
using YawPlanningEvaluator =
    active_3d_planning::trajectory_evaluator::YawPlanningEvaluator;

// Updater specific for yaw planning evaluators. This adaptor allows yaw
// planning segments (which have the yaw planning info struct) to be updated by
// following updaters without further manipulation. Updates only the best view.
class YawPlanningUpdateAdapter : public EvaluatorUpdater {
 public:
  explicit YawPlanningUpdateAdapter(PlannerI& planner);  // NOLINT

  // override virtual functions
  bool updateSegment(TrajectorySegment* segment) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  // factory acces
  static ModuleFactoryRegistry::Registration<YawPlanningUpdateAdapter>
      registration;

  // members
  std::unique_ptr<EvaluatorUpdater> following_updater_;

  // params
  bool p_dynamic_trajectories_;  // true: adapt the trajectory of the best view
                                 // during update, can set false in evaluator
};

// Updater specific for yaw planning evaluators. Updates all orientations stored
// in the info, then selects the best one and applies it to the segment.
class YawPlanningUpdater : public EvaluatorUpdater {
 public:
  explicit YawPlanningUpdater(PlannerI& planner);  // NOLINT

  // override virtual functions
  bool updateSegment(TrajectorySegment* segment) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  // factory acces

  static ModuleFactoryRegistry::Registration<YawPlanningUpdater> registration;

  // members
  std::unique_ptr<EvaluatorUpdater> following_updater_;
  YawPlanningEvaluator* evaluator_;

  // params
  bool p_select_by_value_;
  bool p_dynamic_trajectories_;  // If true recompute the trajectories for
  // segments that have changed
  double
      p_update_range_;    // only update views within this distance of the robot
  double p_update_gain_;  // only update views with a gain above this value
};

}  // namespace evaluator_updater
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_YAW_PLANNING_UPDATERS_H_
