#ifndef MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_CAMERA_MODEL_H
#define MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_CAMERA_MODEL_H

#include "mav_active_3d_planning/trajectory_segment.h"
#include "mav_active_3d_planning/modules/trajectory_evaluators/sensor_models/sensor_model.h"

namespace mav_active_3d_planning {
    namespace sensor_models {

        // Base class for camera models / Utility class that finds visible voxels. Available for all trajectory
        // generators. (improve performance here.)
        class CameraModel : public SensorModel {
        public:
            virtual ~CameraModel() {}

            // Return the voxel centers of all visible voxels for a camera model pointing in x-direction
            virtual bool getVisibleVoxels(std::vector <Eigen::Vector3d> *result, const Eigen::Vector3d &position,
                                          const Eigen::Quaterniond &orientation) = 0;

            // Return the voxel centers of all visible voxels, sampling camera poses from a trajectory segment
            bool
            getVisibleVoxelsFromTrajectory(std::vector <Eigen::Vector3d> *result, const TrajectorySegment &traj_in);

        protected:
            // factory access
            CameraModel() {}

            virtual void setupFromParamMap(Module::ParamMap *param_map);

            virtual bool checkParamsValid(std::string *error_message);

            // parameters
            double p_ray_length_;       // params for camera model
            double p_focal_length_;
            int p_resolution_x_;
            int p_resolution_y_;
            double p_sampling_time_;    // sample camera poses from segment, use 0 for last only

            // constants
            double c_field_of_view_x_;
            double c_field_of_view_y_;

            // For performance timing of different implementations
            bool p_test_;
            std::vector<double> time_count_;
        };

    } // namespace sensor_models
}  // namespace mav_active_3d_planning
#endif //MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_CAMERA_MODEL_H
