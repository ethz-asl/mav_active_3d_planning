#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_SIMULATED_SENSOR_EVALUATOR_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_SIMULATED_SENSOR_EVALUATOR_H

#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/modules/trajectory_evaluators/sensor_models/sensor_model.h"

#include <vector>

namespace mav_active_3d_planning {
    namespace evaluator_updaters{ class SimulatedSensorUpdater; }

    namespace trajectory_evaluators {

        // Base class for a group of evaluators that simulate a sensor in the future to get all visible voxels, then
        // uses these to compute the gain. Can make use of the simulated sensor updater to update the gain of views
        // without further raycasting.
        class SimulatedSensorEvaluator : public TrajectoryEvaluator {
        public:

            // Override virtual functions
            bool computeGain(TrajectorySegment *traj_in);

            virtual void visualizeTrajectoryValue(visualization_msgs::MarkerArray *msg, const TrajectorySegment &trajectory);

        protected:
            friend mav_active_3d_planning::evaluator_updaters::SimulatedSensorUpdater;
            // factory access
            SimulatedSensorEvaluator() {}

            virtual void setupFromParamMap(Module::ParamMap *param_map);

            // members
            std::unique_ptr <SensorModel> sensor_model_;

            // parameters
            bool p_clear_from_parents_;
            bool p_visualize_sensor_view_;

            // store the visible voxel information, asign correct TrajectoryData type here
            virtual bool storeTrajectoryInformation(TrajectorySegment *traj_in,
                                                    const std::vector <Eigen::Vector3d> &new_voxels);

            // compute the gain from Trajectory information
            virtual bool computeGainFromVisibleVoxels(TrajectorySegment *traj_in) = 0;
        };

        // Information struct that is assigned to segments
        struct SimulatedSensorInfo : public TrajectoryInfo {
            virtual ~SimulatedSensorInfo() {}

            std::vector <Eigen::Vector3d> visible_voxels;
        };

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_SIMULATED_SENSOR_EVALUATOR_H
