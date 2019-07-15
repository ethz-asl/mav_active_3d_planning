#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_LINEAR_MAV_TRAJECTORY_GENERATOR_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_LINEAR_MAV_TRAJECTORY_GENERATOR_H

#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include <mav_msgs/eigen_mav_msgs.h>

namespace mav_active_3d_planning {

    // Helper class that creates feasible linear trajectories
    class LinearMavTrajectoryGeneration {
    public:
        LinearMavTrajectoryGeneration() {}

        void setConstraints(double v_max, double a_max, double yaw_rate_max, double sampling_rate);

        // Create feasible trajectory with feasibility checks and smooth derivatives
        bool createTrajectory(const mav_msgs::EigenTrajectoryPoint &start, const mav_msgs::EigenTrajectoryPoint &goal,
                              mav_msgs::EigenTrajectoryPointVector *result);

        // Create a simple representative of a linear segment where only acceleration is included
        bool simulateTrajectory(const mav_msgs::EigenTrajectoryPoint &start, const mav_msgs::EigenTrajectoryPoint &goal,
                                mav_msgs::EigenTrajectoryPointVector *result);

    protected:
        // optimization settings
        mav_trajectory_generation::FeasibilityAnalytic feasibility_check_;
        mav_trajectory_generation::NonlinearOptimizationParameters parameters_;

        // methods
        void optimizeVertices(mav_trajectory_generation::Vertex::Vector *vertices,
                              mav_trajectory_generation::Segment::Vector *segments,
                              mav_trajectory_generation::Trajectory *trajectory);

        bool checkInputFeasible(const mav_trajectory_generation::Segment::Vector &segments);

        // constraints
        double v_max_;
        double a_max_;
        double yaw_rate_max_;
        double sampling_rate_;
    };

}  // namespace mav_active_3d_planning
#endif //MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_LINEAR_MAV_TRAJECTORY_GENERATOR_H

