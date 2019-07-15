#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_NAIVE_EVALUATOR_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_NAIVE_EVALUATOR_H

#include "mav_active_3d_planning/modules/trajectory_evaluators/simulated_sensor_evaluator.h"

namespace mav_active_3d_planning {
    namespace trajectory_evaluators {

        // NaiveEvaluator just counts the number of yet unobserved visible voxels.
        class NaiveEvaluator : public SimulatedSensorEvaluator {
        protected:
            friend ModuleFactory;

            // factory access
            NaiveEvaluator() {}

            void setupFromParamMap(Module::ParamMap *param_map);

            static ModuleFactory::Registration<NaiveEvaluator> registration;

            // Override virtual methods
            bool computeGainFromVisibleVoxels(TrajectorySegment *traj_in);
        };

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_NAIVE_EVALUATOR_H
