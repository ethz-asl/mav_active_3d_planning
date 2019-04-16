#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_FEASIBLE_RRT_STAR_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_FEASIBLE_RRT_STAR_H

#include "mav_active_3d_planning/modules/trajectory_generators/rrt_star.h"


namespace mav_active_3d_planning {
    namespace trajectory_generators {

        // Implementations of the RRT and RRTStar classese, where generated segments obey the system constraints. The
        // velocity at the end of segments is set to 0.0.
        class FeasibleRRT : public RRT {
        protected:
            friend ModuleFactory;
            friend class FeasibleRRTStar;

            // factory access
            FeasibleRRT() {}

            static ModuleFactory::Registration <FeasibleRRT> registration;

            // Overwrite virtual method to create constrained trajectories
            bool connectPoses(const mav_msgs::EigenTrajectoryPoint &start,
                              const mav_msgs::EigenTrajectoryPoint &goal,
                              mav_msgs::EigenTrajectoryPointVector *result);

            // core trajectory generation implementation for both classes
            static bool connectPosesFeasible(const mav_msgs::EigenTrajectoryPoint &start,
                                             const mav_msgs::EigenTrajectoryPoint &goal,
                                             mav_msgs::EigenTrajectoryPointVector *result, double v_max, double a_max,
                                             double yaw_rate_max, double sampling_rate);
        };

        class FeasibleRRTStar : public RRTStar {
        protected:
            friend ModuleFactory;

            // factory access
            FeasibleRRTStar() {}

            static ModuleFactory::Registration <FeasibleRRTStar> registration;

            // Overwrite virtual method to create constrained trajectories
            bool connectPoses(const mav_msgs::EigenTrajectoryPoint &start,
                              const mav_msgs::EigenTrajectoryPoint &goal,
                              mav_msgs::EigenTrajectoryPointVector *result);

        };

    } // namespace trajectory_generators

}  // namespace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_FEASIBLE_RRT_STAR_H
