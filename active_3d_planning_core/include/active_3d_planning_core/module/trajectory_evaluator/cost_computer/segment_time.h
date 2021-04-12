#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_COST_COMPUTER_SEGMENT_TIME_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_COST_COMPUTER_SEGMENT_TIME_H_

#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
namespace cost_computer {

// Execution time of a single segment
class SegmentTime : public CostComputer {
 public:
  explicit SegmentTime(PlannerI& planner);  // NOLINT

  bool computeCost(TrajectorySegment* traj_in) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<SegmentTime> registration;

  // params
  bool p_accumulate_;  // True: Use total time
};

}  // namespace cost_computer
}  // namespace active_3d_planning

#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_COST_COMPUTER_SEGMENT_TIME_H_
