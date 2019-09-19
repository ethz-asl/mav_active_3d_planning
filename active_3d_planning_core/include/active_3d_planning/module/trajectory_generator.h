#ifndef ACTIVE_3D_PLANNING_CORE_TRAJECTORY_GENERATOR_H_
#define ACTIVE_3D_PLANNING_CORE_TRAJECTORY_GENERATOR_H_

#include "active_3d_planning/data/trajectory_segment.h"
#include "active_3d_planning/data/bounding_volume.h"
#include "active_3d_planning/module/module_factory_registry.h"
#include "active_3d_planning/tools/defaults.h"
#include <active_3d_planning/planner/planner_I.h>

#include <Eigen/Core>

#include <string>
#include <vector>


namespace active_3d_planning {

// Forward declaration
class SegmentSelector;

class GeneratorUpdater;


// Base class for trajectory generation to provide uniform interface with other
// classes
class TrajectoryGenerator : public Module {
public:
  TrajectoryGenerator(PlannerI &planner);
  virtual ~TrajectoryGenerator() = default;

  // Expansion policy where to expand (from full tree)
  virtual bool selectSegment(TrajectorySegment **result,
                             TrajectorySegment *root);

  // Expand a selected trajectory segment. Return true for successful expansion.
  virtual bool
  expandSegment(TrajectorySegment *target,
                std::vector<TrajectorySegment *> *new_segments) = 0;

  // Whether and how to update existing segments when a new trajectory is
  // executed
  virtual bool updateSegments(TrajectorySegment *root);

  // Utility function for collision checking. Returns true if the position is
  // reachable.
  bool checkTraversable(const Eigen::Vector3d &position);

  // in case a trajectory needs to be modified to be published
  virtual bool
  extractTrajectoryToPublish(EigenTrajectoryPointVector *trajectory,
                             const TrajectorySegment &segment);

  virtual void setupFromParamMap(Module::ParamMap *param_map) override;

protected:

  // bounding box
  std::unique_ptr<BoundingVolume> bounding_volume_;

  // default modules
  std::unique_ptr<SegmentSelector> segment_selector_;
  std::unique_ptr<GeneratorUpdater> generator_updater_;

  // Parameters
  bool p_collision_optimistic_;
  double p_clearing_radius_; // Unknown space within clearing radius is
                             // considered traversable
  std::string p_selector_args_;
  std::string p_updater_args_;
};

// Abstract encapsulation for default/modular implementations of the
// selectSegment method
class SegmentSelector : public Module {
public:
  SegmentSelector(PlannerI &planner);
  virtual bool selectSegment(TrajectorySegment **result,
                             TrajectorySegment *root) = 0;
};

// Abstract encapsulation for default/modular implementations of the
// updateSegments method
class GeneratorUpdater : public Module {
public:
  GeneratorUpdater(PlannerI &planner);
  virtual bool updateSegments(TrajectorySegment *root) = 0;
};

} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_CORE_TRAJECTORY_GENERATOR_H_
