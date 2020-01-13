#ifndef ACTIVE_3D_PLANNING_CORE_EVALUATOR_UPDATER_UPDATE_NOTHING_H
#define ACTIVE_3D_PLANNING_CORE_EVALUATOR_UPDATER_UPDATE_NOTHING_H

#include "active_3d_planning_core/module/module_factory.h"
#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
    namespace evaluator_updater {

        // Don't perform any specific update operations
        class UpdateNothing : public EvaluatorUpdater {
        public:
            explicit UpdateNothing(PlannerI &planner);

            bool updateSegment(TrajectorySegment *segment) override;

            void setupFromParamMap(Module::ParamMap *param_map) override;

        protected:
            static ModuleFactoryRegistry::Registration<UpdateNothing> registration;
        };

    } // namespace evaluator_updater
} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_CORE_EVALUATOR_UPDATER_UPDATE_NOTHING_H
