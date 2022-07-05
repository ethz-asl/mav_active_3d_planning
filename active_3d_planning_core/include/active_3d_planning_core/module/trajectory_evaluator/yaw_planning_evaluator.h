#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_YAW_PLANNING_EVALUATOR_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_YAW_PLANNING_EVALUATOR_H_

#include <memory>
#include <string>
#include <vector>

#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
namespace evaluator_updater {
class YawPlanningUpdater;
}
namespace trajectory_evaluator {

// Base class for yaw adapting evaluators. Evaluates different yaws for the
// input trajectory and selects the best one. Evaluation and all other
// functionalities are delegated to the folowing evaluator.
class YawPlanningEvaluator : public TrajectoryEvaluator {
 public:
  explicit YawPlanningEvaluator(PlannerI& planner);  // NOLINT

  // Override virtual functions
  bool computeGain(TrajectorySegment* traj_in) override;

  bool computeCost(TrajectorySegment* traj_in) override;

  bool computeValue(TrajectorySegment* traj_in) override;

  int selectNextBest(TrajectorySegment* traj_in) override;

  bool updateSegment(TrajectorySegment* segment) override;

  void visualizeTrajectoryValue(VisualizationMarkers* markers,
                                const TrajectorySegment& trajectory) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

  bool checkParamsValid(std::string* error_message) override;

 protected:
  friend evaluator_updater::YawPlanningUpdater;

  // members
  std::unique_ptr<TrajectoryEvaluator> following_evaluator_;

  // parameters
  int p_n_directions_;
  bool p_select_by_value_;  // false: only evaluate the gain, true: evaluate
  // gain+cost+value
  double p_update_range_;  // Update only gains within this distance (use 0.0 to
  // check all)
  double p_update_gain_;  // Update only gains within above this

  // methods
  virtual double sampleYaw(double original_yaw, int sample) = 0;

  virtual void setTrajectoryYaw(TrajectorySegment* segment, double start_yaw,
                                double target_yaw) = 0;

  virtual void setBestYaw(TrajectorySegment* segment);
};

// Information struct that is assigned to segments
struct YawPlanningInfo : public TrajectoryInfo {
  virtual ~YawPlanningInfo() = default;

  std::vector<TrajectorySegment>
      orientations;  // Keep all orientated segment versions stored
  int active_orientation;
};

}  // namespace trajectory_evaluator
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_YAW_PLANNING_EVALUATOR_H_
