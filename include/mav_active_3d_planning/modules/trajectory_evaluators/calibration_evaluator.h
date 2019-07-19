#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_CALIBRATION_EVALUATOR_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_CALIBRATION_EVALUATOR_H

#include "mav_active_3d_planning/modules/trajectory_evaluators/simulated_sensor_evaluator.h"

namespace mav_active_3d_planning {
    namespace trajectory_evaluators {

        // Uses oomacts trajectory evaluation to compute gain
        class CalibrationEvaluator : public SimulatedSensorEvaluator {
        protected:
            friend ModuleFactory;

            // factory access
            CalibrationEvaluator() {}

            void setupFromParamMap(Module::ParamMap *param_map);

            static ModuleFactory::Registration<CalibrationEvaluator> registration;

            // Override virtual methods
            //this doesnt do anything with voxels, but name was given
            bool computeGainFromVisibleVoxels(TrajectorySegment *traj_in) override;
        private:
            std::string evaluation_service_name;
            ros::ServiceClient evaluation_client;
            ros::NodeHandle node_handle;
        };
    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_CALIBRATION_EVALUATOR_H
