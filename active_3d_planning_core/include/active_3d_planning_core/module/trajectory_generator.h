#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_H_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include <Eigen/Core>

#include "active_3d_planning_core/data/bounding_volume.h"
#include "active_3d_planning_core/data/trajectory_segment.h"
#include "active_3d_planning_core/module/module_factory_registry.h"
#include "active_3d_planning_core/planner/planner_I.h"
#include "active_3d_planning_core/tools/defaults.h"

namespace active_3d_planning {

// Forward declaration
class SegmentSelector;
class GeneratorUpdater;

// Base class for trajectory generation to provide uniform interface with other
// classes
class TrajectoryGenerator : public Module {
 public:
  explicit TrajectoryGenerator(PlannerI& planner);  // NOLINT

  virtual ~TrajectoryGenerator() = default;

  // Expansion policy where to expand (from full tree)
  virtual bool selectSegment(TrajectorySegment** result,
                             TrajectorySegment* root);

  // Expand a selected trajectory segment. Return true for successful expansion.
  virtual bool expandSegment(TrajectorySegment* target,
                             std::vector<TrajectorySegment*>* new_segments) = 0;

  // Update an existing segment when a new trajectory is executed, return true
  // if the segment is to be kept alive, false if it should be removed from the
  // tree
  virtual bool updateSegment(TrajectorySegment* segment);

  // Utility function for collision checking. Returns true if the position is
  // reachable.
  bool checkTraversable(const Eigen::Vector3d& position);

  // Utility function for multi-agent coordination. Returns true if the position is
  // far away from all other goal positions
  bool checkMultiRobotCollision(const Eigen::Vector3d& position);

  // Update the goal(s) of the generator
  bool updateGoals(const std::string& frame_id,
                           const Eigen::Vector3d& pose);

  // in case a trajectory needs to be modified to be published
  virtual bool extractTrajectoryToPublish(
      EigenTrajectoryPointVector* trajectory, const TrajectorySegment& segment);

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  // bounding box
  std::unique_ptr<BoundingVolume> bounding_volume_;

  // default modules
  std::unique_ptr<SegmentSelector> segment_selector_;
  std::unique_ptr<GeneratorUpdater> generator_updater_;

  // Parameters
  bool p_collision_optimistic_;
  double p_clearing_radius_;  // Unknown space within clearing radius is
  // considered traversable
  double p_robot_radius_;     // robot radius for multi-agent collaborative exploration
  std::string p_robot_frame_id_; // the robot frame id for this robot
                                        // to identify which robot this is
  std::string p_selector_args_;
  std::string p_updater_args_;

  // Dictionary to keep track of goals
  std::unique_ptr<std::map<std::string, Eigen::Vector3d>> recent_goal_poses_;
  std::mutex recent_goal_poses_mutex_;
};

// Abstract encapsulation for default/modular implementations of the
// selectSegment method
class SegmentSelector : public Module {
 public:
  explicit SegmentSelector(PlannerI& planner);  // NOLINT

  virtual bool selectSegment(TrajectorySegment** result,
                             TrajectorySegment* root) = 0;
};

// Abstract encapsulation for default/modular implementations of the
// updateSegments method
class GeneratorUpdater : public Module {
 public:
  explicit GeneratorUpdater(PlannerI& planner);  // NOLINT

  virtual bool updateSegment(TrajectorySegment* segment) = 0;
};

}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_H_
