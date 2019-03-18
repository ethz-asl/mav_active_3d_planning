#ifndef MAV_ACTIVE_3D_PLANNING_EVALUATOR_UPDATERS_YAW_PLANNING_UPDATERS_H
#define MAV_ACTIVE_3D_PLANNING_EVALUATOR_UPDATERS_YAW_PLANNING_UPDATERS_H

#include "mav_active_3d_planning/modules/trajectory_evaluators/yaw_planning_evaluators.h"

namespace mav_active_3d_planning {
    class ModuleFactory;

    namespace evaluator_updaters {

        // Updater specific for yaw planning evaluators. This adaptor allows yaw planning segments (which have the
        // yaw planning info struct) to be updated by following updaters without further manipulation).
        class YawPlanningUpdateAdapter : public EvaluatorUpdater {
        public:
            // override virtual functions
            bool updateSegments(TrajectorySegment *root);

        protected:
            friend ModuleFactory;

            // factory acces
            YawPlanningUpdateAdapter() {}
            void setupFromParamMap(Module::ParamMap *param_map);

            // members
            std::unique_ptr<EvaluatorUpdater> following_updater_;

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

            // members
            std::unique_ptr<EvaluatorUpdater> following_updater_;

            // params
            bool p_select_by_value_;

            // methods
            void updateSingle(TrajectorySegment *segment);
        };

    } // namespace evaluator_updaters
} // namepsace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_EVALUATOR_UPDATERS_YAW_PLANNING_UPDATERS_H

