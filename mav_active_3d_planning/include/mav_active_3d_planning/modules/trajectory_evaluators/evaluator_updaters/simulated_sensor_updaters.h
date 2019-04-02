#ifndef MAV_ACTIVE_3D_PLANNING_EVALUATOR_UPDATERS_SIMULATED_SENSOR_UPDATERS_H
#define MAV_ACTIVE_3D_PLANNING_EVALUATOR_UPDATERS_SIMULATED_SENSOR_UPDATERS_H

#include "mav_active_3d_planning/modules/trajectory_evaluators/simulated_sensor_evaluators.h"

namespace mav_active_3d_planning {
    namespace evaluator_updaters {
        using SimulatedSensorEvaluator = mav_active_3d_planning::trajectory_evaluators::SimulatedSensorEvaluator;

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
            SimulatedSensorEvaluator* evaluator_;
        };

        class FrontierUpdater : public EvaluatorUpdater {
        public:
            // override virtual functions
            bool updateSegments(TrajectorySegment *root);

        protected:
            friend ModuleFactory;

            // factory acces
            FrontierUpdater() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            static ModuleFactory::Registration<FrontierUpdater> registration;

            // methods
            void updateSingle(TrajectorySegment *segment);

            // members
            std::unique_ptr<EvaluatorUpdater> following_updater_;
            SimulatedSensorEvaluator* evaluator_ ;
        };

    } // namespace evaluator_updaters
} // namepsace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_EVALUATOR_UPDATERS_SIMULATED_SENSOR_UPDATERS_H

