#ifndef ACTIVE_3D_PLANNING_DIFFDRIVE_TRAJECTORY_GENERATOR_DIFFDRIVE_TRAJECTORY_GENERATION_H
#define ACTIVE_3D_PLANNING_DIFFDRIVE_TRAJECTORY_GENERATOR_DIFFDRIVE_TRAJECTORY_GENERATION_H

#include "active_3d_planning/module/trajectory_generator.h"


namespace active_3d_planning {
namespace trajectory_generator {

// Create random trajectories using a diffdrive model
class DiffDriveTrajectoryGeneration : public TrajectoryGenerator {
public:
  DiffDriveTrajectoryGeneration(PlannerI &planner);
  // override virtual functions
  bool expandSegment(TrajectorySegment *target,
                     std::vector<TrajectorySegment *> *new_segments) override;

  void setupFromParamMap(Module::ParamMap *param_map) override;

  bool checkParamsValid(std::string *error_message) override;

protected:
  static ModuleFactoryRegistry::Registration<DiffDriveTrajectoryGeneration>
      registration;


  // parameters
  double p_distance_max_;
  double p_distance_min_;
  double p_sampling_rate_;
  int p_n_segments_;
  int p_max_tries_;
  double max_dyaw_; //[rad/s]
  double max_vel_;  //[m/s]
  double max_acc_;  //[m/s²]
  double max_time_; //[s]

  bool checkTrajectoryCollision(
      const EigenTrajectoryPoint::Vector &states);
};

} // namespace trajectory_generators
} // namespace mav_active_3d_planning
#endif // ACTIVE_3D_PLANNING_DIFFDRIVE_TRAJECTORY_GENERATOR_DIFFDRIVE_TRAJECTORY_GENERATION_H
