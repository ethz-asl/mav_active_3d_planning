#ifndef ACTIVE_3D_PLANNING_CORE_TRAJECTORY_EVALUATOR_CALIBRATION_EVALUATOR_H
#define ACTIVE_3D_PLANNING_CORE_TRAJECTORY_EVALUATOR_CALIBRATION_EVALUATOR_H

#include "active_3d_planning/module/trajectory_evaluator/simulated_sensor_evaluator.h"

namespace active_3d_planning {
namespace trajectory_evaluator {

// Uses oomacts trajectory evaluation to compute gain
class CalibrationEvaluator : public SimulatedSensorEvaluator {
public:
  // factory access
  CalibrationEvaluator(PlannerI &planner);

  void setupFromParamMap(Module::ParamMap *param_map) override;

protected:
  static ModuleFactoryRegistry::Registration<CalibrationEvaluator> registration;

  // Override virtual methods
  // this doesnt do anything with voxels, but name was given
  bool computeGainFromVisibleVoxels(TrajectorySegment *traj_in) override;

private:
  std::string evaluation_service_name;
  ros::ServiceClient evaluation_client;
  ros::Subscriber pathCompletionListener;
  ros::NodeHandle node_handle;
  int idCounter = 0;
  std::unordered_map<TrajectorySegment *, int> trajIdMap;

  void resetMap(const std_msgs::Empty &msg);
};
} // namespace trajectory_evaluator
} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_CORE_TRAJECTORY_EVALUATOR_CALIBRATION_EVALUATOR_H
