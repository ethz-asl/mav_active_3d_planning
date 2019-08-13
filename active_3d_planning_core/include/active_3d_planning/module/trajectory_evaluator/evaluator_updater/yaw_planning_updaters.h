#ifndef ACTIVE_3D_PLANNING_CORE_EVALUATOR_UPDATER_YAW_PLANNING_UPDATERS_H
#define ACTIVE_3D_PLANNING_CORE_EVALUATOR_UPDATER_YAW_PLANNING_UPDATERS_H

#include "active_3d_planning/module/trajectory_evaluator/yaw_planning_evaluator.h"

namespace active_3d_planning {
namespace evaluator_updater {
using YawPlanningEvaluator =
    active_3d_planning::trajectory_evaluator::YawPlanningEvaluator;

// Updater specific for yaw planning evaluators. This adaptor allows yaw
// planning segments (which have the yaw planning info struct) to be updated by
// following updaters without further manipulation. Updates only the best view.
class YawPlanningUpdateAdapter : public EvaluatorUpdater {
public:
  YawPlanningUpdateAdapter(PlannerI &planner);

  // override virtual functions
  bool updateSegments(TrajectorySegment *root) override;

  void setupFromParamMap(Module::ParamMap *param_map) override;

protected:
  // factory acces
  static ModuleFactoryRegistry::Registration<YawPlanningUpdateAdapter>
      registration;

  // members
  std::unique_ptr<EvaluatorUpdater> following_updater_;

  // params
  bool p_dynamic_trajectories_; // true: adapt the trajectory of the best view
                                // during update, can set false in evaluator

  // methods
  void updateSingle(TrajectorySegment *segment);
};

// Updater specific for yaw planning evaluators. Updates all orientations stored
// in the info, then selects the best one and applies it to the segment.
class YawPlanningUpdater : public EvaluatorUpdater {
public:
  YawPlanningUpdater(PlannerI &planner);
  // override virtual functions
  bool updateSegments(TrajectorySegment *root) override;

  void setupFromParamMap(Module::ParamMap *param_map) override;

protected:
  // factory acces

  static ModuleFactoryRegistry::Registration<YawPlanningUpdater> registration;

  // members
  std::unique_ptr<EvaluatorUpdater> following_updater_;

  // params
  bool p_select_by_value_;
  bool p_dynamic_trajectories_; // If true recompute the trajectories for
                                // segments that have changed
  double p_update_range_; // maximum distance to the robot a view needs to have
                          // to be updated
  double p_update_gain_;  // minimum gain a view needs to be updated

  // methods
  void updateSingle(TrajectorySegment *segment);
};

} // namespace evaluator_updater
} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_CORE_EVALUATOR_UPDATER_YAW_PLANNING_UPDATERS_H
