#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_SIMPLE_YAW_PLANNING_EVALUATOR_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_SIMPLE_YAW_PLANNING_EVALUATOR_H

#include "mav_active_3d_planning/modules/trajectory_evaluators/yaw_planning_evaluator.h"


namespace mav_active_3d_planning {
    namespace evaluator_updaters { class YawPlanningUpdater; }
    namespace trajectory_evaluators {

        // Simple evaluator. Samples yaws uniformly and assigns the same yaw to all trajectory points.
        class SimpleYawPlanningEvaluator : public YawPlanningEvaluator {
        public:

            // Override virtual functions
            void visualizeTrajectoryValue(visualization_msgs::MarkerArray* msg, const TrajectorySegment &trajectory);

        protected:
            friend ModuleFactory;
            friend class mav_active_3d_planning::evaluator_updaters::YawPlanningUpdater;

            // factory access
            SimpleYawPlanningEvaluator() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            static ModuleFactory::Registration<SimpleYawPlanningEvaluator> registration;

            // params
            bool p_visualize_followup_;     // true: also visualize the gain of the best orientation

            // methods
            double sampleYaw(double original_yaw, int sample);
            void setTrajectoryYaw(TrajectorySegment* segment, double start_yaw, double target_yaw);
        };

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_SIMPLE_YAW_PLANNING_EVALUATOR_H
