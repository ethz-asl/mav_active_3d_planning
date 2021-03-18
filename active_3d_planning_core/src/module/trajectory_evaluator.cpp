#include "active_3d_planning_core/module/trajectory_evaluator.h"

#include <string>

#include <active_3d_planning_core/module/module_factory.h>
#include <active_3d_planning_core/planner/planner_I.h>

namespace active_3d_planning {

TrajectoryEvaluator::TrajectoryEvaluator(PlannerI& planner) : Module(planner) {}

void TrajectoryEvaluator::setupFromParamMap(Module::ParamMap* param_map) {
  // Get the args to build the modules, default is a namespace extension
  std::string ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "cost_computer_args", &p_cost_args_,
                        ns + "/cost_computer");
  setParam<std::string>(param_map, "value_computer_args", &p_value_args_,
                        ns + "/value_computer");
  setParam<std::string>(param_map, "next_selector_args", &p_next_args_,
                        ns + "/next_selector");
  setParam<std::string>(param_map, "evaluator_updater_args", &p_updater_args_,
                        ns + "/evaluator_updater");
  std::string bounding_volume_args;
  setParam<std::string>(param_map, "bounding_volume_args",
                        &bounding_volume_args, ns + "/bounding_volume");
  bounding_volume_ = planner_.getFactory().createModule<BoundingVolume>(
      bounding_volume_args, planner_, verbose_modules_);
}

bool TrajectoryEvaluator::computeCost(TrajectorySegment* traj_in) {
  // If not implemented use a (default) module
  if (!cost_computer_) {
    cost_computer_ = planner_.getFactory().createModule<CostComputer>(
        p_cost_args_, planner_, verbose_modules_);
  }
  return cost_computer_->computeCost(traj_in);
}

bool TrajectoryEvaluator::computeValue(TrajectorySegment* traj_in) {
  // If not implemented use a (default) module
  if (!value_computer_) {
    value_computer_ = planner_.getFactory().createModule<ValueComputer>(
        p_value_args_, planner_, verbose_modules_);
  }
  return value_computer_->computeValue(traj_in);
}

int TrajectoryEvaluator::selectNextBest(TrajectorySegment* traj_in) {
  // If not implemented use a (default) module
  if (!next_selector_) {
    next_selector_ = planner_.getFactory().createModule<NextSelector>(
        p_next_args_, planner_, verbose_modules_);
  }
  return next_selector_->selectNextBest(traj_in);
}

bool TrajectoryEvaluator::updateSegment(TrajectorySegment* segment) {
  // If not implemented use a (default) module
  if (!evaluator_updater_) {
    evaluator_updater_ = planner_.getFactory().createModule<EvaluatorUpdater>(
        p_updater_args_, planner_, verbose_modules_);
  }
  return evaluator_updater_->updateSegment(segment);
}

}  // namespace active_3d_planning
