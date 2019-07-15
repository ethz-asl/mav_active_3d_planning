#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_VOXEL_TYPE_EVALUATOR_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_VOXEL_TYPE_EVALUATOR_H

#include "mav_active_3d_planning/modules/trajectory_evaluators/simulated_sensor_evaluator.h"

namespace mav_active_3d_planning {
    namespace trajectory_evaluators {

        // VoxelType assigns a constant value to every kind of visible value (unobserved/free/occupied, inside/outside
        // target bounding volume)
        class VoxelTypeEvaluator : public SimulatedSensorEvaluator {
        protected:
            friend ModuleFactory;

            // factory access
            VoxelTypeEvaluator() {}

            void setupFromParamMap(Module::ParamMap *param_map);

            static ModuleFactory::Registration<VoxelTypeEvaluator> registration;

            // members
            std::unique_ptr<defaults::BoundingVolume> outer_volume_;

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
#endif // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_VOXEL_TYPE_EVALUATOR_H
