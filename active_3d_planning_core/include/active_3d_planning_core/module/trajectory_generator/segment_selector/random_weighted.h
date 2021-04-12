#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_SEGMENT_SELECTOR_RANDOM_WEIGHTED_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_SEGMENT_SELECTOR_RANDOM_WEIGHTED_H_

#include <string>

#include "active_3d_planning_core/module/trajectory_generator.h"

namespace active_3d_planning {
namespace segment_selector {

// Select segments at random, weighted with their value
class RandomWeighted : public SegmentSelector {
 public:
  explicit RandomWeighted(PlannerI& planner);  // NOLINT

  // override virtual functions
  bool selectSegment(TrajectorySegment** result,
                     TrajectorySegment* root) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

  bool checkParamsValid(std::string* error_message) override;

 protected:
  static ModuleFactoryRegistry::Registration<RandomWeighted> registration;

  // params
  double
      leaf_probability_;  // Probability to select a leaf (set to 1 for leaves
  // only)
  bool revisit_;                // only unchecked segments
  double factor_;               // weighting (potency), 0 for uniform
  double uniform_probability_;  // part of probability mass that is distributed
  // uniformly (to guarantee non-zero probability for all candidates)
};

}  // namespace segment_selector
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_SEGMENT_SELECTOR_RANDOM_WEIGHTED_H_
