#ifndef ACTIVE_3D_PLANNING_CORE_VALUE_COMPUTER_ACCUMULATE_VALUE_H
#define ACTIVE_3D_PLANNING_CORE_VALUE_COMPUTER_ACCUMULATE_VALUE_H

#include "active_3d_planning_core/module/module_factory_registry.h"
#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
    namespace value_computer {

        // Accumulates the values up to the root, use another value computer for
        // individual values (Decorator pattern)
        class AccumulateValue : public ValueComputer {
        public:
            AccumulateValue(PlannerI &planner);

            // override virtual functions
            bool computeValue(TrajectorySegment *traj_in) override;

            void setupFromParamMap(Module::ParamMap *param_map) override;

        protected:
            static ModuleFactoryRegistry::Registration<AccumulateValue> registration;

            // members
            std::unique_ptr<ValueComputer> following_value_computer_;
        };

    } // namespace value_computer
} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_CORE_VALUE_COMPUTER_ACCUMULATE_VALUE_H
