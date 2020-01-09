#ifndef ACTIVE_3D_PLANNING_CORE_TRAJECTORY_EVALUATOR_NAIVE_EVALUATOR_H
#define ACTIVE_3D_PLANNING_CORE_TRAJECTORY_EVALUATOR_NAIVE_EVALUATOR_H

#include "active_3d_planning/module/trajectory_evaluator/simulated_sensor_evaluator.h"

namespace active_3d_planning {
    namespace trajectory_evaluator {

// NaiveEvaluator just counts the number of yet unobserved visible voxels.
        class NaiveEvaluator : public SimulatedSensorEvaluator {
        public:
            NaiveEvaluator(PlannerI &planner);

            void setupFromParamMap(Module::ParamMap *param_map) override;

        protected:
            static ModuleFactoryRegistry::Registration<NaiveEvaluator> registration;

            // Override virtual methods
            bool computeGainFromVisibleVoxels(TrajectorySegment *traj_in) override;
        };

    } // namespace trajectory_evaluator
} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_CORE_TRAJECTORY_EVALUATOR_NAIVE_EVALUATOR_H
