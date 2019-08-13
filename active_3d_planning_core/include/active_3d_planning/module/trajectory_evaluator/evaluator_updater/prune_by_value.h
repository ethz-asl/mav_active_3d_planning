#ifndef ACTIVE_3D_PLANNING_EVALUATOR_UPDATER_PRUNE_BY_VALUE_H
#define ACTIVE_3D_PLANNING_EVALUATOR_UPDATER_PRUNE_BY_VALUE_H

#include "active_3d_planning/module/module_factory.h"
#include "active_3d_planning/module/trajectory_evaluator.h"

namespace active_3d_planning {
namespace evaluator_updater {

// Remove all segments that dont have a minimum value, can then call another
// updater
class PruneByValue : public EvaluatorUpdater {
public:
  PruneByValue(PlannerI &planner);
  bool updateSegments(TrajectorySegment *root) override;
  void setupFromParamMap(Module::ParamMap *param_map) override;

protected:
  static ModuleFactoryRegistry::Registration<PruneByValue> registration;

  // methods
  bool removeSingle(TrajectorySegment *segment, double min_value);
  double getMinimumValue(TrajectorySegment *segment);

  // params
  double minimum_value_;     // value below which segments are cropped
  bool use_relative_values_; // if true scale all existing values to 0-1
  bool include_subsequent_;  // if true keep all segments where at least 1 child
                             // matches the min value
  std::unique_ptr<EvaluatorUpdater> following_updater_;
};

} // namespace evaluator_updater
} // namespace active_3d_planning

#endif // ACTIVE_3D_PLANNING_EVALUATOR_UPDATER_PRUNE_BY_VALUE_H
