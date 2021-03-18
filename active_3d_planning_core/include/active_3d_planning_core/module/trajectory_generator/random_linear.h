#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_RANDOM_LINEAR_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_RANDOM_LINEAR_H_

#include <string>
#include <vector>

#include "active_3d_planning_core/module/trajectory_generator.h"

namespace active_3d_planning {
namespace trajectory_generator {

// Point sampling in space, joined by linear segments
class RandomLinear : public TrajectoryGenerator {
 public:
  explicit RandomLinear(PlannerI& planner);  // NOLINT

  // override virtual functions
  bool expandSegment(TrajectorySegment* target,
                     std::vector<TrajectorySegment*>* new_segments) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

  bool checkParamsValid(std::string* error_message) override;

 protected:
  static ModuleFactoryRegistry::Registration<RandomLinear> registration;

  // params
  double p_min_distance_;   // m
  double p_max_distance_;   // m
  double p_v_max_;          // m/s
  double p_a_max_;          // m/s2
  double p_sampling_rate_;  // Hz
  int p_n_segments_;
  int p_max_tries_;
  bool p_planar_;
  bool p_sample_yaw_;  // false: face direction of travel

  // methods
  void sampleTarget(Eigen::Vector3d* direction, double* yaw,
                    double* decelleration_distance);

  bool buildTrajectory(const Eigen::Vector3d& start_pos,
                       const Eigen::Vector3d& direction, double yaw,
                       double decelleration_distance,
                       TrajectorySegment* new_segment);
};

}  // namespace trajectory_generator
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_RANDOM_LINEAR_H_
