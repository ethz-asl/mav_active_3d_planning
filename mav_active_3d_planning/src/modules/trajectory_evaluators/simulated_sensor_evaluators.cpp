#include "mav_active_3d_planning/modules/trajectory_evaluators/simulated_sensor_evaluators.h"

#include <algorithm>

namespace mav_active_3d_planning {
    namespace trajectory_evaluators {

        // SimulatedSensorEvaluator (base class)
        void SimulatedSensorEvaluator::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<bool>(param_map, "clear_from_parents", &p_clear_from_parents_, false);

            // Create sensor model
            std::string args;   // default args extends the parent namespace
            std::string param_ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "sensor_model_args", &args,
                                  param_ns + "/sensor_model");
            sensor_model_ = ModuleFactory::Instance()->createModule<SensorModel>(args, verbose_modules_, voxblox_ptr_);

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
        }

        // NaiveEvaluator
        ModuleFactory::Registration<NaiveEvaluator> NaiveEvaluator::registration("NaiveEvaluator");

        void NaiveEvaluator::setupFromParamMap(Module::ParamMap *param_map) {
            // setup parent
            SimulatedSensorEvaluator::setupFromParamMap(param_map);
        }

        bool NaiveEvaluator::computeGainFromVisibleVoxels(TrajectorySegment *traj_in) {
            if (!traj_in->info) {
                traj_in->gain = 0.0;
                return false;
            }
            // remove all already observed voxels, count number of new voxels
            SimulatedSensorInfo *info = dynamic_cast<SimulatedSensorInfo *>(traj_in->info.get());
            info->visible_voxels.erase(
                    std::remove_if(info->visible_voxels.begin(), info->visible_voxels.end(),
                                   [this](const Eigen::Vector3d &voxel) {
                                       return voxblox_ptr_->getEsdfMapPtr()->isObserved(voxel);
                                   }), info->visible_voxels.end());
            traj_in->gain = (double) info->visible_voxels.size();
            return true;
        }

        // Frontier
        ModuleFactory::Registration<Frontier> Frontier::registration("Frontier");

        void Frontier::setupFromParamMap(Module::ParamMap *param_map) {
            // setup parent
            SimulatedSensorEvaluator::setupFromParamMap(param_map);
        }

        bool Frontier::isFrontierVoxel(const Eigen::Vector3d &voxel) {
            double voxel_size = static_cast<double>(voxblox_ptr_->getEsdfMapPtr()->voxel_size());
            voxblox::EsdfMap *esdf_map = voxblox_ptr_->getEsdfMapPtr().get();

            // Check all neighboring voxels
            double distance;
            if (esdf_map->getDistanceAtPosition(voxel + Eigen::Vector3d(voxel_size, 0, 0), &distance)) {
                if (distance < voxel_size) { return true; }
            }
            if (esdf_map->getDistanceAtPosition(voxel - Eigen::Vector3d(voxel_size, 0, 0), &distance)) {
                if (distance < voxel_size) { return true; }
            }
            if (esdf_map->getDistanceAtPosition(voxel + Eigen::Vector3d(0, voxel_size, 0), &distance)) {
                if (distance < voxel_size) { return true; }
            }
            if (esdf_map->getDistanceAtPosition(voxel - Eigen::Vector3d(0, voxel_size, 0), &distance)) {
                if (distance < voxel_size) { return true; }
            }
            if (esdf_map->getDistanceAtPosition(voxel + Eigen::Vector3d(0, 0, voxel_size), &distance)) {
                if (distance < voxel_size) { return true; }
            }
            if (esdf_map->getDistanceAtPosition(voxel - Eigen::Vector3d(0, 0, voxel_size), &distance)) {
                if (distance < voxel_size) { return true; }
            }
            return false;
        }

        bool Frontier::storeTrajectoryInformation(TrajectorySegment *traj_in,
                                                  const std::vector <Eigen::Vector3d> &new_voxels) {
            FrontierInfo *new_info = new FrontierInfo();
            new_info->visible_voxels.assign(new_voxels.begin(), new_voxels.end());
            traj_in->info.reset(new_info);
            return true;
        }

        bool Frontier::computeGainFromVisibleVoxels(TrajectorySegment *traj_in) {
            if (!traj_in->info) {
                traj_in->gain = 0.0;
                return false;
            }
            FrontierInfo *info = dynamic_cast<FrontierInfo *>(traj_in->info.get());

            // Remove observed voxels
            info->visible_voxels.erase(
                    std::remove_if(info->visible_voxels.begin(), info->visible_voxels.end(),
                                   [this](const Eigen::Vector3d &voxel) {
                                       return voxblox_ptr_->getEsdfMapPtr()->isObserved(voxel);
                                   }),
                    info->visible_voxels.end());

            // Check which voxels are frontier voxels
            info->frontier_voxels.clear();
            for (int i = 0; i < info->visible_voxels.size(); ++i) {
                if (isFrontierVoxel(info->visible_voxels[i])) {
                    info->frontier_voxels.push_back(info->visible_voxels[i]);
                }
            }
            traj_in->gain = (double) info->frontier_voxels.size();
            return true;
        }

        void Frontier::visualizeTrajectoryValue(visualization_msgs::MarkerArray *msg, const TrajectorySegment &trajectory) {
            // Default implementation displays all frontier voxels
            if (!trajectory.info) { return; }
            visualization_msgs::Marker new_msg;
            new_msg.header.frame_id = "/world";
            new_msg.ns = "evaluation";
            new_msg.header.stamp = ros::Time::now();
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
            new_msg.color.a = 1.0;

            // points
            FrontierInfo *info = dynamic_cast<FrontierInfo *>(trajectory.info.get());
            for (int i = 0; i < info->frontier_voxels.size(); ++i) {
                geometry_msgs::Point point;
                point.x = (double) info->frontier_voxels[i].x();
                point.y = (double) info->frontier_voxels[i].y();
                point.z = (double) info->frontier_voxels[i].z();
                new_msg.points.push_back(point);
            }
            msg->markers.push_back(new_msg);
        }

        // VoxelType
        ModuleFactory::Registration<VoxelType> VoxelType::registration("VoxelType");

        void VoxelType::setupFromParamMap(Module::ParamMap *param_map) {
            // params
            setParam<double>(param_map, "gain_unknown", &p_gain_unknown_, 1.0);
            setParam<double>(param_map, "gain_occupied", &p_gain_occupied_, 0.0);
            setParam<double>(param_map, "gain_free", &p_gain_free_, 0.0);
            setParam<double>(param_map, "gain_unknown_outer", &p_gain_unknown_outer_, 0.0);
            setParam<double>(param_map, "gain_occupied_outer", &p_gain_occupied_outer_, 0.0);
            setParam<double>(param_map, "gain_free_outer", &p_gain_free_outer_, 0.0);

            // outer volume (if not specified will not be counted)
            std::string ns = (*param_map)["param_namespace"];
            std::string outer_volume_args;
            setParam<std::string>(param_map, "outer_volume_args", &outer_volume_args, ns + "/outer_volume");
            outer_volume_ = ModuleFactory::Instance()->createModule<defaults::BoundingVolume>(outer_volume_args,
                                                                                              verbose_modules_);

            // constants
            c_min_gain_ = std::min({p_gain_unknown_, p_gain_occupied_, p_gain_free_, p_gain_unknown_outer_,
                                    p_gain_occupied_outer_, p_gain_free_outer_});
            c_max_gain_ = std::max({p_gain_unknown_, p_gain_occupied_, p_gain_free_, p_gain_unknown_outer_,
                                    p_gain_occupied_outer_, p_gain_free_outer_});

            // setup parent
            SimulatedSensorEvaluator::setupFromParamMap(param_map);
        }

        bool VoxelType::computeGainFromVisibleVoxels(TrajectorySegment *traj_in) {
            traj_in->gain = 0.0;
            if (!traj_in->info) { return false; }
            SimulatedSensorInfo *info = dynamic_cast<SimulatedSensorInfo *>(traj_in->info.get());
            for (int i = 0; i < info->visible_voxels.size(); ++i) {
                double distance = 0.0;
                if (bounding_volume_->contains(info->visible_voxels[i])) {
                    if (voxblox_ptr_->getEsdfMapPtr()->getDistanceAtPosition(info->visible_voxels[i], &distance)) {
                        if (distance < 0.0) {
                            traj_in->gain += p_gain_occupied_;
                        } else {
                            traj_in->gain += p_gain_free_;
                        }
                    } else {
                        traj_in->gain += p_gain_unknown_;
                    }
                } else {
                    if (voxblox_ptr_->getEsdfMapPtr()->getDistanceAtPosition(info->visible_voxels[i],
                                                                             &distance)) {
                        if (distance < 0.0) {
                            traj_in->gain += p_gain_occupied_outer_;
                        } else {
                            traj_in->gain += p_gain_free_outer_;
                        }
                    } else {
                        traj_in->gain += p_gain_unknown_outer_;
                    }
                }
            }
            return true;
        }

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
