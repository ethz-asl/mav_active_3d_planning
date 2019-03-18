#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_SIMULATED_SENSOR_EVALUATOR_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_SIMULATED_SENSOR_EVALUATOR_H

#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/modules/trajectory_evaluators/sensor_models/sensor_model.h"

#include <vector>

namespace mav_active_3d_planning {
    class ModuleFactory;
    namespace evaluator_updaters{ class SimulatedSensorUpdater; }

    namespace trajectory_evaluators {

        // Base class for a group of evaluators that simulate a sensor in the future to get all visible voxels, then
        // uses these to compute the gain.
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

        // NaiveEvaluator just counts the number of yet unobserved visible voxels.
        class NaiveEvaluator : public SimulatedSensorEvaluator {
        protected:
            friend ModuleFactory;

            // factory access
            NaiveEvaluator() {}

            void setupFromParamMap(Module::ParamMap *param_map);

            // Override virtual methods
            bool computeGainFromVisibleVoxels(TrajectorySegment *traj_in);
        };

        // Frontier counts all unknown voxels which are next to known voxels
        class Frontier : public SimulatedSensorEvaluator {
        public:
            // Override virtual methods
            void visualizeTrajectoryValue(visualization_msgs::MarkerArray *msg, const TrajectorySegment &trajectory);

        protected:
            friend ModuleFactory;

            // factory access
            Frontier() {}

            void setupFromParamMap(Module::ParamMap *param_map);

            // Override virtual methods
            bool storeTrajectoryInformation(TrajectorySegment *traj_in,
                                            const std::vector <Eigen::Vector3d> &new_voxels);

            bool computeGainFromVisibleVoxels(TrajectorySegment *traj_in);

            // methods
            bool isFrontierVoxel(const Eigen::Vector3d &voxel);
        };

        // Information struct that is assigned to segments
        struct FrontierInfo : public SimulatedSensorInfo {
            std::vector <Eigen::Vector3d> frontier_voxels;
        };

        // VoxelType assigns a constant value to every kind of visible value (unobserved, free, occupied, ...)
        class VoxelType : public SimulatedSensorEvaluator {
        protected:
            friend ModuleFactory;

            // factory access
            VoxelType() {}

            void setupFromParamMap(Module::ParamMap *param_map);

            // members
            defaults::BoundingVolume outer_volume_;

            // parameters
            double p_gain_unknown_;
            double p_gain_unknown_outer_;
            double p_gain_occupied_;
            double p_gain_occupied_outer_;
            double p_gain_free_;
            double p_gain_free_outer_;

            // constants
            double c_min_gain_;
            double c_max_gain_;

            // Override virtual methods
            bool computeGainFromVisibleVoxels(TrajectorySegment *traj_in);
        };

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_SIMULATED_SENSOR_EVALUATOR_H
