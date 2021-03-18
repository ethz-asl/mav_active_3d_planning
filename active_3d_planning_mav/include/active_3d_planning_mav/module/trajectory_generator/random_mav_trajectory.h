#ifndef ACTIVE_3D_PLANNING_MAV_MODULE_TRAJECTORY_GENERATOR_RANDOM_MAV_TRAJECTORY_H_
#define ACTIVE_3D_PLANNING_MAV_MODULE_TRAJECTORY_GENERATOR_RANDOM_MAV_TRAJECTORY_H_

#include <string>
#include <vector>

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>

#include "active_3d_planning_core/module/trajectory_generator.h"

namespace active_3d_planning {
namespace trajectory_generator {

// Create random trajectories using the mav_trajectory_generation tools and
// check for MAV constraints (such as max yaw rate, thrusts, ...)
class RandomMavTrajectory : public TrajectoryGenerator {
 public:
  explicit RandomMavTrajectory(PlannerI& planner);  // NOLINT

  // override virtual functions
  bool expandSegment(TrajectorySegment* target,
                     std::vector<TrajectorySegment*>* new_segments) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

  bool checkParamsValid(std::string* error_message) override;

 protected:
  static ModuleFactoryRegistry::Registration<RandomMavTrajectory> registration;

  // methods
  void initializeConstraints();

  // parameters
  double p_distance_max_;
  double p_distance_min_;
  double p_sampling_rate_;
  int p_n_segments_;
  int p_max_tries_;

  // optimization settings
  mav_trajectory_generation::FeasibilityAnalytic feasibility_check_;
  mav_trajectory_generation::NonlinearOptimizationParameters parameters_;

  // methods
  void sampleGoalPose(double* yaw, Eigen::Vector3d* goal_pos,
                      const Eigen::Vector3d& start_pos);

  void optimizeVertices(mav_trajectory_generation::Vertex::Vector* vertices,
                        mav_trajectory_generation::Segment::Vector* segments,
                        mav_trajectory_generation::Trajectory* trajectory);

  bool checkInputFeasible(
      const mav_trajectory_generation::Segment::Vector& segments);

  bool checkTrajectoryCollision(const EigenTrajectoryPoint::Vector& states);
};

}  // namespace trajectory_generator
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_MAV_MODULE_TRAJECTORY_GENERATOR_RANDOM_MAV_TRAJECTORY_H_
