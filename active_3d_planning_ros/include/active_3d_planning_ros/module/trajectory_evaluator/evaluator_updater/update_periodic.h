#ifndef ACTIVE_3D_PLANNING_ROS_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_UPDATE_PERIODIC_H_
#define ACTIVE_3D_PLANNING_ROS_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_UPDATE_PERIODIC_H_

#include <memory>

#include <ros/time.h>

#include "active_3d_planning_core/module/module_factory_registry.h"
#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
namespace evaluator_updater {

// Only periodically call another updater
class UpdatePeriodic : public EvaluatorUpdater {
 public:
  explicit UpdatePeriodic(PlannerI& planner);  // NOLINT

  // override virtual functions
  bool updateSegment(TrajectorySegment* segment) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<UpdatePeriodic> registration;

  // params
  double p_minimum_wait_time_;
  int p_minimum_wait_calls_;

  // variables
  ros::Time previous_time_;  // Need to use simulated time here
  int waited_calls_;
  bool update_;                       // whether or not to update the tree
  TrajectorySegment* previous_root_;  // keep track of teh root

  // members
  std::unique_ptr<EvaluatorUpdater> following_updater_;
};

}  // namespace evaluator_updater
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_ROS_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_UPDATE_PERIODIC_H_
