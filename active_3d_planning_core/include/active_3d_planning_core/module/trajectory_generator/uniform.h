#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_UNIFORM_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_UNIFORM_H_

#include <vector>

#include "active_3d_planning_core/module/trajectory_generator.h"

namespace active_3d_planning {
namespace trajectory_generator {

// Sample the input space and propose a uniform grid of new trajectories
class Uniform : public TrajectoryGenerator {
 public:
  explicit Uniform(PlannerI& planner);  // NOLINT

  // override virtual functions
  bool expandSegment(TrajectorySegment* target,
                     std::vector<TrajectorySegment*>* new_segments) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<Uniform> registration;

  // parameters
  double p_distance_;       // m
  double p_velocity_;       // m/s
  double p_yaw_angle_;      // rad
  double p_sampling_rate_;  // Hz
  int p_n_segments_;
  double p_ascent_angle_;  // Rad, Set 0 for planar

  // constants
  double c_yaw_rate_;
};

}  // namespace trajectory_generator
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_UNIFORM_H_
