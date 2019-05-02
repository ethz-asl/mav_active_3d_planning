#ifndef MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_ITERATIVE_RAY_CASTER_LIDAR_H
#define MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_ITERATIVE_RAY_CASTER_LIDAR_H

#include "mav_active_3d_planning/modules/trajectory_evaluators/sensor_models/lidar_model.h"


namespace mav_active_3d_planning {
    namespace sensor_models {

        // Cast Rays until they span 1 full voxel, then duplicate them to omit redundancies
        // (timed at 42 +/- 20 ms)
        class IterativeRayCasterLidar : public LidarModel {
        public:
            virtual ~IterativeRayCasterLidar() {}

            // Override virtual functions
            bool getVisibleVoxels(std::vector <Eigen::Vector3d> *result, const Eigen::Vector3d &position,
                                  const Eigen::Quaterniond &orientation);

        protected:
            friend ModuleFactory;

            // factory access
            IterativeRayCasterLidar() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            static ModuleFactory::Registration<IterativeRayCasterLidar> registration;

            // params
            double p_ray_step_;
            double p_downsampling_factor_;      // Artificially reduce the minimum resolution to increase performance

            // constants
            int c_res_x_;       // factual resolution that is used for ray casting
            int c_res_y_;
            int c_n_sections_;     // number of ray duplications
            std::vector<double> c_split_distances_;      // distances where rays are duplicated
            std::vector<int> c_split_widths_;            // number of max distance rays that are covered per split

            // variables
            Eigen::ArrayXXi ray_table_;

            // methods
            void markNeighboringRays(int x, int y, int segment, int value);
        };

    } // namespace sensor_models
}  // namespace mav_active_3d_planning
#endif //MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_ITERATIVE_RAY_CASTER_LIDAR_H
