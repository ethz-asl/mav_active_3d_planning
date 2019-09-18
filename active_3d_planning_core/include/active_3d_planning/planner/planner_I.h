#ifndef ACTIVE_3D_PLANNING_CORE_PLANNER_I_H_
#define ACTIVE_3D_PLANNING_CORE_PLANNER_I_H_

#include <Eigen/Dense>

namespace voxblox {
class EsdfMap;
class TsdfMap;
} // namespace voxblox
namespace active_3d_planning {

class BackTracker;
class TrajectoryGenerator;
class TrajectoryEvaluator;
class ModuleFactory;
//struct VisualizationMarkers;


// basic interface of planners for module-access
class PlannerI {
public:
  virtual ~PlannerI() = default;

  virtual const Eigen::Vector3d &getCurrentPosition() const = 0;
  virtual const Eigen::Quaterniond &getCurrentOrientation() const = 0;

  virtual BackTracker &getBackTracker() = 0;
  virtual TrajectoryGenerator &getTrajectoryGenerator() = 0;
  virtual TrajectoryEvaluator &getTrajectoryEvaluator() = 0;

//  virtual void publishVisualization(const VisualizationMarkers& markers) = 0;

  virtual ModuleFactory &getFactory() = 0;

  // maybe want to get rid of voxblox at some point
  virtual voxblox::EsdfMap &getMap() = 0;
  virtual voxblox::TsdfMap &getTsdfMap() = 0;
};
} // namespace active_3d_planning

#endif /* ACTIVE_3D_PLANNING_CORE_PLANNER_I_H_ */