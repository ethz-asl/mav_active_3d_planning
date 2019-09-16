#ifndef ACTIVE_3D_PLANNING_CORE_TRAJECTORY_EVALUATOR_H
#define ACTIVE_3D_PLANNING_CORE_TRAJECTORY_EVALUATOR_H

#include <active_3d_planning/data/trajectory_segment.h>
#include <active_3d_planning/module/module_factory_registry.h>
#include <active_3d_planning/tools/defaults.h>

// TODO get voxblox out
#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <voxblox_ros/esdf_server.h>

#include <memory>
#include <string>

namespace active_3d_planning {

// Forward declaration
class CostComputer;
class ValueComputer;
class NextSelector;
class EvaluatorUpdater;

// Base class for trajectory evaluators to provide uniform interface with other
// classes
class TrajectoryEvaluator : public Module {
public:
  TrajectoryEvaluator(PlannerI &planner);
  virtual ~TrajectoryEvaluator() = default;

  // compute the gain of a TrajectorySegment
  virtual bool computeGain(TrajectorySegment *traj_in) = 0;

  // compute the cost of a TrajectorySegment
  virtual bool computeCost(TrajectorySegment *traj_in);

  // compute the Value of a segment with known cost and gain
  virtual bool computeValue(TrajectorySegment *traj_in);

  // return the index of the most promising child segment
  virtual int selectNextBest(TrajectorySegment *traj_in);

  // Whether and how to update existing segments when a new trajectory is
  // executed
  virtual bool updateSegments(TrajectorySegment *root);

  // Implement this method to allow visualization of the information gain during
  // simulation
  virtual void visualizeTrajectoryValue(visualization_msgs::MarkerArray *msg,
                                        const TrajectorySegment &trajectory) {}

  virtual void setupFromParamMap(Module::ParamMap *param_map);
protected:
  // Voxblox map
  voxblox::EsdfServer &voxblox_;

  // bounding volume of interesting target
  std::unique_ptr<defaults::BoundingVolume> bounding_volume_;

  // params
  std::string p_cost_args_;
  std::string p_value_args_;
  std::string p_next_args_;
  std::string p_updater_args_;

  // default modules
  std::unique_ptr<CostComputer> cost_computer_;
  std::unique_ptr<ValueComputer> value_computer_;
  std::unique_ptr<NextSelector> next_selector_;
  std::unique_ptr<EvaluatorUpdater> evaluator_updater_;
};

// Abstract encapsulation for default/modular implementations of the computeCost
// method
class CostComputer : public Module {
public:
  CostComputer(PlannerI &planner) : Module(planner) {}
  virtual bool computeCost(TrajectorySegment *traj_in) = 0;
};

// Abstract encapsulation for default/modular implementations of the
// computeValue method
class ValueComputer : public Module {
public:
  ValueComputer(PlannerI &planner) : Module(planner) {}
  virtual bool computeValue(TrajectorySegment *traj_in) = 0;
};

// Abstract encapsulation for default/modular implementations of the
// selectNextBest method
class NextSelector : public Module {
public:
  NextSelector(PlannerI &planner) : Module(planner) {}
  virtual int selectNextBest(TrajectorySegment *traj_in) = 0;
};

// Abstract encapsulation for default/modular implementations of the
// updateSegments method
class EvaluatorUpdater : public Module {
public:
  EvaluatorUpdater(PlannerI &planner) : Module(planner) {}
  virtual bool updateSegments(TrajectorySegment *root) = 0;
};

} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_CORE_TRAJECTORY_EVALUATOR_H
