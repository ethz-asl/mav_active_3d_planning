#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_FEASIBLE_RRT_STAR_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_FEASIBLE_RRT_STAR_H

#include "mav_active_3d_planning/modules/trajectory_generators/rrt_star.h"
#include "mav_active_3d_planning/modules/back_trackers/default_back_trackers.h"

#include "mav_active_3d_planning/modules/trajectory_generators/linear_mav_trajectory_generator.h"


namespace mav_active_3d_planning {
    // Implementations of the RRT, RRTStar and RotateReverse classes, where segments are created using the
    // mav_trajectory_generation package to obey the system constraints and create smooth derivatives. The
    // velocity at the end of each segment is set to 0.0.

    namespace trajectory_generators {

        class FeasibleRRT : public RRT {
        protected:
            friend ModuleFactory;

            // factory access
            FeasibleRRT() {}

            static ModuleFactory::Registration <FeasibleRRT> registration;

            virtual void setupFromParamMap(Module::ParamMap *param_map);

            // Segment creator
            LinearMavTrajectoryGeneration segment_generator_;

            // params
            bool p_all_semgents_feasible_;

            // Overwrite virtual method to create constrained trajectories
            bool connectPoses(const mav_msgs::EigenTrajectoryPoint &start,
                              const mav_msgs::EigenTrajectoryPoint &goal,
                              mav_msgs::EigenTrajectoryPointVector *result);

            bool extractTrajectoryToPublish(mav_msgs::EigenTrajectoryPointVector *trajectory,
                                            const TrajectorySegment &segment);
        };

        class FeasibleRRTStar : public RRTStar {
        protected:
            friend ModuleFactory;

            // factory access
            FeasibleRRTStar() {}

            static ModuleFactory::Registration <FeasibleRRTStar> registration;

            virtual void setupFromParamMap(Module::ParamMap *param_map);

            // Segment creator
            LinearMavTrajectoryGeneration segment_generator_;

            // params
            bool p_all_semgents_feasible_;

            // Overwrite virtual method to create constrained trajectories
            bool connectPoses(const mav_msgs::EigenTrajectoryPoint &start,
                              const mav_msgs::EigenTrajectoryPoint &goal,
                              mav_msgs::EigenTrajectoryPointVector *result);

            bool extractTrajectoryToPublish(mav_msgs::EigenTrajectoryPointVector *trajectory,
                                            const TrajectorySegment &segment);
        };

    } // namespace trajectory_generators


    namespace back_trackers {

        // Try rotating in place, if nothing found reverse most recent segments
        class FeasibleRotateReverse : public RotateReverse {
        protected:
            friend ModuleFactory;

            // factory access
            FeasibleRotateReverse() {}

            void setupFromParamMap(Module::ParamMap *param_map);

            static ModuleFactory::Registration <FeasibleRotateReverse> registration;

            // Segment creator
            LinearMavTrajectoryGeneration segment_generator_;

            // Overwrite virtual methods
            bool rotate(TrajectorySegment *target);
        };

    } // namespace back_trackers

}  // namespace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_FEASIBLE_RRT_STAR_H
