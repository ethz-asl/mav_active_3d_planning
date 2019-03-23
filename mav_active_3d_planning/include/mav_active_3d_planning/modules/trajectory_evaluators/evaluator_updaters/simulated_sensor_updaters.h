#ifndef MAV_ACTIVE_3D_PLANNING_EVALUATOR_UPDATERS_SIMULATED_SENSOR_UPDATERS_H
#define MAV_ACTIVE_3D_PLANNING_EVALUATOR_UPDATERS_SIMULATED_SENSOR_UPDATERS_H

#include "mav_active_3d_planning/modules/trajectory_evaluators/simulated_sensor_evaluators.h"

namespace mav_active_3d_planning {
    class ModuleFactory;
    namespace evaluator_updaters {

        // Updater specific for simulated sensor evaluators. Calls the evaluators computeGainFromVisibleVoxels function
        // -> no need for raycasting again.
        class SimulatedSensorUpdater : public EvaluatorUpdater {
        public:
            // override virtual functions
            bool updateSegments(TrajectorySegment *root);

        protected:
            friend ModuleFactory;

            // factory acces
            SimulatedSensorUpdater() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            static ModuleFactory::Registration<SimulatedSensorUpdater> registration;

            // methods
            void updateSingle(TrajectorySegment *segment);

            // members
            std::unique_ptr<EvaluatorUpdater> following_updater_;
        };

    } // namespace evaluator_updaters
} // namepsace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_EVALUATOR_UPDATERS_SIMULATED_SENSOR_UPDATERS_H

