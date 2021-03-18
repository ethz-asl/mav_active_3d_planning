#include "active_3d_planning_ros/module/trajectory_evaluator/evaluator_updater/update_periodic.h"

#include <string>

#include "active_3d_planning_core/module/module_factory.h"
#include "active_3d_planning_core/planner/planner_I.h"

namespace active_3d_planning {
namespace evaluator_updater {

// UpdatePeriodic
ModuleFactoryRegistry::Registration<UpdatePeriodic>
    UpdatePeriodic::registration("UpdatePeriodic");

UpdatePeriodic::UpdatePeriodic(PlannerI& planner) : EvaluatorUpdater(planner) {}

bool UpdatePeriodic::updateSegment(TrajectorySegment* segment) {
  if (!(segment->parent->parent)) {
    // at root
    if (segment->parent != previous_root_) {
      previous_root_ = segment->parent;
      // Both conditions need to be met
      if ((ros::Time::now() - previous_time_).toSec() >= p_minimum_wait_time_ &&
          waited_calls_ >= p_minimum_wait_calls_) {
        previous_time_ = ros::Time::now();
        waited_calls_ = 0;
        update_ = true;
      } else {
        waited_calls_++;
        update_ = false;
      }
    }
  }
  if (update_) {
    return following_updater_->updateSegment(segment);
  }
  return true;
}

void UpdatePeriodic::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<double>(param_map, "minimum_wait_time", &p_minimum_wait_time_, 0.0);
  setParam<int>(param_map, "minimum_wait_calls", &p_minimum_wait_calls_, 0);
  waited_calls_ = 0;
  previous_time_ = ros::Time::now();

  // Create Following updater (default does nothing)
  std::string args;  // default args extends the parent namespace
  std::string param_ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "following_updater_args", &args,
                        param_ns + "/following_updater");
  following_updater_ = planner_.getFactory().createModule<EvaluatorUpdater>(
      args, planner_, verbose_modules_);
}

}  // namespace evaluator_updater
}  // namespace active_3d_planning
