#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_CONTINUOUS_YAW_PLANNING_EVALUATOR_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_CONTINUOUS_YAW_PLANNING_EVALUATOR_H

#include "mav_active_3d_planning/modules/trajectory_evaluators/yaw_planning_evaluator.h"
#include "mav_active_3d_planning/planner_node.h"


namespace mav_active_3d_planning {
    namespace evaluator_updaters { class YawPlanningUpdater; }
    namespace trajectory_evaluators {

        // Split full yaw into sections and find best orientation by conbining sections. From this paper:
        // https://ieeexplore.ieee.org/abstract/document/8594502
        // Samples yaws uniformly and assigns the same yaw to all trajectory points. Make sure the horizontal FOV of the
        // sensor model matches the spacing of the evaluator
        class ContinuousYawPlanningEvaluator : public YawPlanningEvaluator {
        public:

            // Override virtual functions
            bool computeGain(TrajectorySegment *traj_in) override;
            bool updateSegments(TrajectorySegment *root) override;
            void visualizeTrajectoryValue(visualization_msgs::MarkerArray* msg, const TrajectorySegment &trajectory) override;

        protected:
            friend ModuleFactory;
            friend class mav_active_3d_planning::evaluator_updaters::YawPlanningUpdater;

            // factory access
            ContinuousYawPlanningEvaluator() {}
            void setupFromParamMap(Module::ParamMap *param_map) override;
            static ModuleFactory::Registration<ContinuousYawPlanningEvaluator> registration;

            // params
            bool p_visualize_followup_;     // true: also visualize the gain of the best orientation
            int p_n_sections_fov_;          // Number of sections visible, i.e. fov/section_width
            double p_update_range_;         // Update only gains within this distance (use 0.0 to check all)
            double p_update_gain_;          // Update only gains within above this
            bool p_update_separate_segments_;   // True: check for each section, false: check for whole segment

            // variables
            PlannerNode* planner_node_;     // ref to main planner for distance checks

            // members


            // methods
            double sampleYaw(double original_yaw, int sample) override;
            void setTrajectoryYaw(TrajectorySegment* segment, double start_yaw, double target_yaw) override;
            void setBestYaw(TrajectorySegment* segment);
            bool updateSingle(TrajectorySegment *segment);
        };

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_CONTINUOUS_YAW_PLANNING_EVALUATOR_H
