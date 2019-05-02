#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_FEASIBLE_RRT_STAR_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_FEASIBLE_RRT_STAR_H

#include "mav_active_3d_planning/modules/trajectory_generators/rrt_star.h"

#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>


namespace mav_active_3d_planning {
    namespace trajectory_generators {

        // Implementations of the RRTStar classes, where generated segments obey the system constraints. The
        // velocity at the end of segments is set to 0.0.

        class FeasibleRRTStar : public RRTStar {
        protected:
            friend ModuleFactory;

            // factory access
            FeasibleRRTStar() {}

            static ModuleFactory::Registration <FeasibleRRTStar> registration;

            virtual void setupFromParamMap(Module::ParamMap *param_map);

            // Overwrite virtual method to create constrained trajectories
            bool connectPoses(const mav_msgs::EigenTrajectoryPoint &start,
                              const mav_msgs::EigenTrajectoryPoint &goal,
                              mav_msgs::EigenTrajectoryPointVector *result);

            // optimization settings
            mav_trajectory_generation::FeasibilityAnalytic feasibility_check_;
            mav_trajectory_generation::NonlinearOptimizationParameters parameters_;

            // methods
            void optimizeVertices(mav_trajectory_generation::Vertex::Vector *vertices,
                                  mav_trajectory_generation::Segment::Vector *segments,
                                  mav_trajectory_generation::Trajectory *trajectory);

            bool checkInputFeasible(const mav_trajectory_generation::Segment::Vector &segments);

            bool checkTrajectoryCollision(const mav_msgs::EigenTrajectoryPoint::Vector &states);

            bool connectPosesFeasible(const mav_msgs::EigenTrajectoryPoint &start,
                                      const mav_msgs::EigenTrajectoryPoint &goal,
                                      mav_msgs::EigenTrajectoryPointVector *result, double v_max, double a_max,
                                      double yaw_rate_max, double sampling_rate);
        };

    } // namespace trajectory_generators

}  // namespace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_FEASIBLE_RRT_STAR_H
