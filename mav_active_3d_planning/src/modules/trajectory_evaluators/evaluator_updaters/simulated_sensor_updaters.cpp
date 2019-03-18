#include "mav_active_3d_planning/modules/trajectory_evaluators/evaluator_updaters/simulated_sensor_updaters.h"
#include "mav_active_3d_planning/module_factory.h"


namespace mav_active_3d_planning {
    namespace evaluator_updaters {
        using SimulatedSensorEvaluator = mav_active_3d_planning::trajectory_evaluators::SimulatedSensorEvaluator;


        // SimulatedSensorUpdater
        bool SimulatedSensorUpdater::updateSegments(TrajectorySegment *root) {
            // recursively update all segments from root to leaves (as in the planner)
            updateSingle(root);
            return following_updater_->updateSegments(root);
        }

        void SimulatedSensorUpdater::setupFromParamMap(Module::ParamMap *param_map) {
            // Create Following updater (default does nothing)
            std::string args;   // default args extends the parent namespace
            std::string param_ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "following_updater_args", &args, param_ns + "/following_updater");
            following_updater_ = ModuleFactory::Instance()->createEvaluatorUpdater(args, parent_, verbose_modules_);
        }

        void SimulatedSensorUpdater::updateSingle(TrajectorySegment *segment) {
            // call the parent to reevaluate the voxels
            dynamic_cast<SimulatedSensorEvaluator *>(parent_)->computeGainFromVisibleVoxels(segment);
            for (int i = 0; i < segment->children.size(); ++i) {
                updateSingle(segment->children[i].get());
            }
        }

    } // namespace evaluator_updaters
} // namepsace mav_active_3d_planning
