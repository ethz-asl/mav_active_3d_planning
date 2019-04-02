#ifndef MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_SENSOR_MODEL_H
#define MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_SENSOR_MODEL_H

#include "mav_active_3d_planning/trajectory_segment.h"
#include "mav_active_3d_planning/module_factory.h"

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

        // Implement this function to allow visualization of the sensing bounds
        virtual void
        visualizeSensorView(visualization_msgs::MarkerArray *msg, const TrajectorySegment &trajectory) {}

    protected:
        friend ModuleFactory;

        // factory access
        SensorModel() {}

        virtual void setupFromParamMap(Module::ParamMap *param_map) {}

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
