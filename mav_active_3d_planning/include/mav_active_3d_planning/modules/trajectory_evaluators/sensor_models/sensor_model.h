#ifndef MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_SENSOR_MODEL_H
#define MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_SENSOR_MODEL_H

#include "mav_active_3d_planning/trajectory_segment.h"
#include "mav_active_3d_planning/module.h"

#include <voxblox_ros/esdf_server.h>
#include <Eigen/Core>

#include <string>
#include <memory>

namespace mav_active_3d_planning {

        // Base class that interfaces with sensor based evaluators.
        class SensorModel : public Module {
        public:
            virtual ~SensorModel() {}

            // Return the voxel centers of all visible voxels
            virtual bool getVisibleVoxelsFromTrajectory(std::vector <Eigen::Vector3d> *result,
                                                        const TrajectorySegment &traj_in) = 0;

        protected:
            friend class ModuleFactory;

            // factory access
            SensorModel() {}

            virtual void setupFromParamMap(Module::ParamMap *param_map) {}

            // override this function if voxblox specific initialization is needed, call parent implementation fist!
            void setVoxbloxPtr(const std::shared_ptr <voxblox::EsdfServer> &voxblox_ptr);

            // voxblox map
            std::shared_ptr <voxblox::EsdfServer> voxblox_ptr_;

            // constants
            voxblox::FloatingPoint c_voxel_size_;
            voxblox::FloatingPoint c_block_size_;

            // changes the input point to the center of the corresponding voxel
            bool getVoxelCenter(Eigen::Vector3d *point);
        };

}  // namespace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_SENSOR_MODEL_H
