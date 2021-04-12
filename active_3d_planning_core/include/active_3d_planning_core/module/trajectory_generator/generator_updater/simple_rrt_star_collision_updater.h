#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_GENERATOR_UPDATER_SIMPLE_RRT_STAR_COLLISION_UPDATER_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_GENERATOR_UPDATER_SIMPLE_RRT_STAR_COLLISION_UPDATER_H_

#include <memory>
#include <vector>

#include "active_3d_planning_core/module/trajectory_generator/rrt_star.h"

namespace active_3d_planning {
namespace generator_updater {

// Recursively check whether the trajectories are still collision free, if not
// try to rewire them
class SimpleRRTStarCollisionUpdater : public GeneratorUpdater {
 public:
  explicit SimpleRRTStarCollisionUpdater(PlannerI& planner);  // NOLINT

  // override virtual functions
  bool updateSegment(TrajectorySegment* segment) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<SimpleRRTStarCollisionUpdater>
      registration;

  // members
  std::unique_ptr<GeneratorUpdater> following_updater_;
  // reference to Generator for rewiring
  trajectory_generator::RRTStar* generator_;

  // methods
  bool isCollided(const EigenTrajectoryPointVector& trajectory);

  void checkSingle(TrajectorySegment* segment,
                   std::vector<TrajectorySegment*>* to_rewire,
                   std::vector<TrajectorySegment*>* safe_segments);

  // variables
  TrajectorySegment* previous_root_;
};

}  // namespace generator_updater
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_GENERATOR_UPDATER_SIMPLE_RRT_STAR_COLLISION_UPDATER_H_
