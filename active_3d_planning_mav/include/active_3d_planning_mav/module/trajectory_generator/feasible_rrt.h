#ifndef ACTIVE_3D_PLANNING_MAV_MODULE_TRAJECTORY_GENERATOR_FEASIBLE_RRT_H_
#define ACTIVE_3D_PLANNING_MAV_MODULE_TRAJECTORY_GENERATOR_FEASIBLE_RRT_H_

#include "active_3d_planning_core/module/trajectory_generator/rrt_star.h"
#include "active_3d_planning_mav/tools/linear_mav_trajectory_generator.h"

namespace active_3d_planning {
namespace trajectory_generator {

class FeasibleRRT : public RRT {
 public:
  explicit FeasibleRRT(PlannerI& planner);  // NOLINT

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<FeasibleRRT> registration;

  // Segment creator
  LinearMavTrajectoryGenerator segment_generator_;

  // params
  bool p_all_semgents_feasible_;

  // Overwrite virtual method to create constrained trajectories
  bool connectPoses(const EigenTrajectoryPoint& start,
                    const EigenTrajectoryPoint& goal,
                    EigenTrajectoryPointVector* result,
                    bool check_collision = true) override;

  bool extractTrajectoryToPublish(EigenTrajectoryPointVector* trajectory,
                                  const TrajectorySegment& segment) override;
};

}  // namespace trajectory_generator
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_MAV_MODULE_TRAJECTORY_GENERATOR_FEASIBLE_RRT_H_
