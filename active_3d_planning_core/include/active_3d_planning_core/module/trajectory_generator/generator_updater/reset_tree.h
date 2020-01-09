#ifndef ACTIVE_3D_PLANNING_CORE_GENERATOR_UPDATER_DEFAULT_GENERATOR_UPDATERS_H
#define ACTIVE_3D_PLANNING_CORE_GENERATOR_UPDATER_DEFAULT_GENERATOR_UPDATERS_H

#include "active_3d_planning_core/module/trajectory_generator.h"

namespace active_3d_planning {
    namespace generator_updater {

// Discard all segments and start from scratch
        class ResetTree : public GeneratorUpdater {
        public:
            explicit ResetTree(PlannerI &planner);

            // override virtual functions
            bool updateSegment(TrajectorySegment *segment) override;

            void setupFromParamMap(Module::ParamMap *param_map) override;

        protected:
            static ModuleFactoryRegistry::Registration<ResetTree> registration;
        };

    } // namespace generator_updater
} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_CORE_GENERATOR_UPDATER_DEFAULT_GENERATOR_UPDATERS_H
