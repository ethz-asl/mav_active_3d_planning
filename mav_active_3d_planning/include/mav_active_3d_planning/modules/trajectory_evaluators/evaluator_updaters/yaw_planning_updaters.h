#ifndef MAV_ACTIVE_3D_PLANNING_EVALUATOR_UPDATERS_YAW_PLANNING_UPDATERS_H
#define MAV_ACTIVE_3D_PLANNING_EVALUATOR_UPDATERS_YAW_PLANNING_UPDATERS_H

#include "mav_active_3d_planning/modules/trajectory_evaluators/yaw_planning_evaluators.h"

namespace mav_active_3d_planning {
    namespace evaluator_updaters {
        using YawPlanningEvaluator = mav_active_3d_planning::trajectory_evaluators::YawPlanningEvaluator;

        // Updater specific for yaw planning evaluators. This adaptor allows yaw planning segments (which have the
        // yaw planning info struct) to be updated by following updaters without further manipulation. Updates only the
        // best view.
        class YawPlanningUpdateAdapter : public EvaluatorUpdater {
        public:
            // override virtual functions
            bool updateSegments(TrajectorySegment *root);

        protected:
            friend ModuleFactory;

            // factory acces
            YawPlanningUpdateAdapter() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            static ModuleFactory::Registration<YawPlanningUpdateAdapter> registration;

            // members
            std::unique_ptr<EvaluatorUpdater> following_updater_;

            // params
            bool p_dynamic_trajectories_;   // true: adapt the trajectory of the best view during update, can set false in evaluator

            // methods
            void updateSingle(TrajectorySegment *segment);
        };

        // Updater specific for yaw planning evaluators. Updates all orientations stored in the info, then selects the
        // best one and applies it to the segment.
        class YawPlanningUpdater : public EvaluatorUpdater {
        public:
            // override virtual functions
            bool updateSegments(TrajectorySegment *root);

        protected:
            friend ModuleFactory;

            // factory acces
            YawPlanningUpdater() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            static ModuleFactory::Registration<YawPlanningUpdater> registration;

            // members
            std::unique_ptr<EvaluatorUpdater> following_updater_;
            std::unique_ptr<EvaluatorUpdater> view_updater_;
            YawPlanningEvaluator* evaluator_;

            // params
            bool p_select_by_value_;
            bool p_dynamic_trajectories_;   // If true recompute the trajectories for segments that have changed

            // methods
            void updateSingle(TrajectorySegment *segment);
        };

    } // namespace evaluator_updaters
} // namepsace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_EVALUATOR_UPDATERS_YAW_PLANNING_UPDATERS_H

