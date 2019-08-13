#ifndef ACTIVE_3D_PLANNING_ROS_EVALUATOR_UPDATER_UPDATE_PERIODIC_H
#define ACTIVE_3D_PLANNING_ROS_EVALUATOR_UPDATER_UPDATE_PERIODIC_H

#include <active_3d_planning/module/module_factory_registry.h>
#include "active_3d_planning/module/trajectory_evaluator.h"

#include <ros/time.h>

#include <memory>

namespace active_3d_planning {
namespace evaluator_updater {
// Only periodically call another updater
class UpdatePeriodic : public EvaluatorUpdater {
public:
  UpdatePeriodic(PlannerI &planner);
  // override virtual functions
  bool updateSegments(TrajectorySegment *root) override;

  void setupFromParamMap(Module::ParamMap *param_map) override;

protected:
  static ModuleFactoryRegistry::Registration<UpdatePeriodic> registration;

  // params
  double p_minimum_wait_time_;
  int p_minimum_wait_calls_;

  // variables
  ros::Time previous_time_; // Need to use simulated time here
  int waited_calls_;

  // members
  std::unique_ptr<EvaluatorUpdater> following_updater_;
};

} // namespace evaluator_updater
} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_ROS_EVALUATOR_UPDATER_UPDATE_PERIODIC_H
