#ifndef ACTIVE_3D_PLANNING_CORE_VALUE_COMPUTER_DEFAULT_VALUE_COMPUTERS_H
#define ACTIVE_3D_PLANNING_CORE_VALUE_COMPUTER_DEFAULT_VALUE_COMPUTERS_H

#include "active_3d_planning_core/module/module_factory_registry.h"
#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
    namespace value_computer {

        // Linear combination of cost and gain
        class LinearValue : public ValueComputer {
        public:
            LinearValue(PlannerI &planner);

            // override virtual functions
            bool computeValue(TrajectorySegment *traj_in) override;

            void setupFromParamMap(Module::ParamMap *param_map) override;

        protected:
            static ModuleFactoryRegistry::Registration<LinearValue> registration;

            // params
            double cost_weight_;
            double gain_weight_;
            bool p_accumulate_cost_; // If true first accumulate all cost, then discount
            bool p_accumulate_gain_; // If true first accumulate all cost, then discount
        };

    } // namespace value_computer
} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_CORE_VALUE_COMPUTER_DEFAULT_VALUE_COMPUTERS_H
