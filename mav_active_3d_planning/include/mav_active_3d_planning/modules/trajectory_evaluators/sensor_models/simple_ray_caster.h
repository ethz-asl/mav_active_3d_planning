#ifndef MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_SIMPLE_RAY_CASTER_H
#define MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_SIMPLE_RAY_CASTER_H

#include "mav_active_3d_planning/trajectory_segment.h"
#include "mav_active_3d_planning/defaults.h"
#include "mav_active_3d_planning/modules/trajectory_evaluators/sensor_models/camera_model.h"


namespace mav_active_3d_planning {
    namespace sensor_models {

        // Get minimum resolution reuqired for 1 ray per voxel at max distance, then cast a ray through every
        // such voxel to detect visible voxels (timed at 90 +/- 30 ms)
        class SimpleRayCaster : public CameraModel {
        public:
            virtual ~SimpleRayCaster() {}

            // Override virutal functions
            bool getVisibleVoxels(std::vector <Eigen::Vector3d> *result, const Eigen::Vector3d &position,
                                  const Eigen::Quaterniond &orientation);

        protected:
            friend ModuleFactory;

            // factory access
            SimpleRayCaster() {}

            void setupFromParamMap(Module::ParamMap *param_map);

            static ModuleFactory::Registration<SimpleRayCaster> registration;

            // params
            double p_ray_step_;

            // constants
            int c_res_x_;       // factual resolution that is used for ray casting
            int c_res_y_;
        };

    } // namespace sensor_models
}  // namespace mav_active_3d_planning
#endif //MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_SIMPLE_RAY_CASTER_H
