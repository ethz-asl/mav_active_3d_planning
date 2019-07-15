#include "mav_active_3d_planning/modules/trajectory_evaluators/simulated_sensor_evaluator.h"

#include <algorithm>

namespace mav_active_3d_planning {
    namespace trajectory_evaluators {

        // SimulatedSensorEvaluator (base class)
        void SimulatedSensorEvaluator::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<bool>(param_map, "clear_from_parents", &p_clear_from_parents_, false);
            setParam<bool>(param_map, "visualize_sensor_view", &p_visualize_sensor_view_, false);

            // Register link for simulated sensor udpaters
            ModuleFactory::Instance()->registerLinkableModule("SimulatedSensorEvaluator", this);

            // Create sensor model
            std::string args;   // default args extends the parent namespace
            std::string param_ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "sensor_model_args", &args,
                                  param_ns + "/sensor_model");
            sensor_model_ = ModuleFactory::Instance()->createModule<SensorModel>(args, verbose_modules_);

            // setup parent
            TrajectoryEvaluator::setupFromParamMap(param_map);
        }

        bool SimulatedSensorEvaluator::computeGain(TrajectorySegment *traj_in) {
            std::vector <Eigen::Vector3d> new_voxels;
            sensor_model_->getVisibleVoxelsFromTrajectory(&new_voxels, *traj_in);

            // Check for interesting bounding box
            if (bounding_volume_->is_setup) {
                new_voxels.erase(std::remove_if(new_voxels.begin(), new_voxels.end(),
                                                [this](const Eigen::Vector3d &voxel) {
                                                    return (!bounding_volume_->contains(voxel));
                                                }), new_voxels.end());
            }

            // Remove voxels previously seen by parent (this is quite expensive)
            if (p_clear_from_parents_) {
                TrajectorySegment *previous = traj_in->parent;
                while (previous) {
                    std::vector <Eigen::Vector3d> old_voxels =
                            dynamic_cast<SimulatedSensorInfo *>(previous->info.get())->visible_voxels;
                    new_voxels.erase(std::remove_if(new_voxels.begin(), new_voxels.end(),
                                                    [&old_voxels](const Eigen::Vector3d &voxel) {
                                                        auto it = std::find(old_voxels.begin(), old_voxels.end(),
                                                                            voxel);
                                                        return (it != old_voxels.end());
                                                    }), new_voxels.end());
                    previous = previous->parent;
                }
            }

            // Compute gain and store trajectory information
            storeTrajectoryInformation(traj_in, new_voxels);
            computeGainFromVisibleVoxels(traj_in);
            return true;
        }

        bool SimulatedSensorEvaluator::storeTrajectoryInformation(TrajectorySegment *traj_in,
                                                                  const std::vector <Eigen::Vector3d> &new_voxels) {
            SimulatedSensorInfo *new_info = new SimulatedSensorInfo();
            new_info->visible_voxels.assign(new_voxels.begin(), new_voxels.end());
            traj_in->info.reset(new_info);
            return true;
        }

        // Base Visualization: just display all visible voxels
        void SimulatedSensorEvaluator::visualizeTrajectoryValue(visualization_msgs::MarkerArray *msg,
                                                                const TrajectorySegment &trajectory) {
            if (!trajectory.info) { return; }
            // Default implementation displays all visible voxels
            visualization_msgs::Marker new_msg;
            new_msg.ns = "evaluation";
            new_msg.header.stamp = ros::Time::now();
            new_msg.header.frame_id = "/world";
            new_msg.id = defaults::getNextVisualizationId(*msg);
            new_msg.pose.orientation.w = 1.0;
            new_msg.type = visualization_msgs::Marker::CUBE_LIST;
            voxblox::FloatingPoint voxel_size = voxblox_ptr_->getEsdfMapPtr()->voxel_size();
            new_msg.scale.x = (double) voxel_size;
            new_msg.scale.y = (double) voxel_size;
            new_msg.scale.z = (double) voxel_size;
            new_msg.color.r = 1.0;
            new_msg.color.g = 0.8;
            new_msg.color.b = 0.0;
            new_msg.color.a = 0.4;

            // points
            SimulatedSensorInfo *info = dynamic_cast<SimulatedSensorInfo *>(trajectory.info.get());
            for (int i = 0; i < info->visible_voxels.size(); ++i) {
                geometry_msgs::Point point;
                point.x = (double) info->visible_voxels[i].x();
                point.y = (double) info->visible_voxels[i].y();
                point.z = (double) info->visible_voxels[i].z();
                new_msg.points.push_back(point);
            }
            msg->markers.push_back(new_msg);

            if (p_visualize_sensor_view_) {
                sensor_model_->visualizeSensorView(msg, trajectory);
            }
        }

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
