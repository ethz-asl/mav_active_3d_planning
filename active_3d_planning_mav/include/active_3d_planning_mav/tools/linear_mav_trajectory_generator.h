#ifndef ACTIVE_3D_PLANNING_MAV_TOOLS_LINEAR_MAV_TRAJECTORY_GENERATOR_H_
#define ACTIVE_3D_PLANNING_MAV_TOOLS_LINEAR_MAV_TRAJECTORY_GENERATOR_H_

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>

#include "active_3d_planning_core/data/trajectory.h"

namespace active_3d_planning {

// Helper class that creates feasible linear trajectories
class LinearMavTrajectoryGenerator {
 public:
  LinearMavTrajectoryGenerator() = default;

  void setConstraints(double v_max, double a_max, double yaw_rate_max,
                      double yaw_accel_max, double sampling_rate);

  // Create feasible trajectory with feasibility checks and smooth derivatives
  bool createTrajectory(const EigenTrajectoryPoint& start,
                        const EigenTrajectoryPoint& goal,
                        EigenTrajectoryPointVector* result);

  // Create a simple representative of a linear segment where only acceleration
  // is included
  bool simulateTrajectory(const EigenTrajectoryPoint& start,
                          const EigenTrajectoryPoint& goal,
                          EigenTrajectoryPointVector* result);

 protected:
  // optimization settings
  mav_trajectory_generation::FeasibilityAnalytic feasibility_check_;
  mav_trajectory_generation::NonlinearOptimizationParameters parameters_;

  // methods
  void optimizeVertices(mav_trajectory_generation::Vertex::Vector* vertices,
                        mav_trajectory_generation::Segment::Vector* segments,
                        mav_trajectory_generation::Trajectory* trajectory);

  bool checkInputFeasible(
      const mav_trajectory_generation::Segment::Vector& segments);

  // constraints
  double v_max_;
  double a_max_;
  double yaw_rate_max_;
  double yaw_accel_max_;
  double sampling_rate_;
};

}  // namespace active_3d_planning

#endif  // ACTIVE_3D_PLANNING_MAV_TOOLS_LINEAR_MAV_TRAJECTORY_GENERATOR_H_
