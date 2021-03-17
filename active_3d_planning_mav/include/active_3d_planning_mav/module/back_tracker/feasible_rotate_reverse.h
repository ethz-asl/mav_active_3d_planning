#ifndef ACTIVE_3D_PLANNING_MAV_MODULE_BACK_TRACKER_FEASIBLE_ROTATE_REVERSE_H_
#define ACTIVE_3D_PLANNING_MAV_MODULE_BACK_TRACKER_FEASIBLE_ROTATE_REVERSE_H_

#include "active_3d_planning_core/module/back_tracker/rotate_reverse.h"
#include "active_3d_planning_mav/tools/linear_mav_trajectory_generator.h"

namespace active_3d_planning {
namespace back_tracker {

// Try rotating in place, if nothing found reverse most recent segments
class FeasibleRotateReverse : public RotateReverse {
 public:
  explicit FeasibleRotateReverse(PlannerI& planner);  // NOLINT

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<FeasibleRotateReverse>
      registration;

  // Segment creator
  LinearMavTrajectoryGenerator segment_generator_;

  // Overwrite virtual methods
  bool rotate(TrajectorySegment* target) override;
};

}  // namespace back_tracker
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_MAV_MODULE_BACK_TRACKER_FEASIBLE_ROTATE_REVERSE_H_
