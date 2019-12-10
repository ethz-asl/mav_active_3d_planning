#include "mav_active_3d_planning/modules/trajectory_evaluators/frontier_evaluator.h"

#include <algorithm>

namespace mav_active_3d_planning {
    namespace trajectory_evaluators {

        // Register to factory
        ModuleFactory::Registration<FrontierEvaluator> FrontierEvaluator::registration("FrontierEvaluator");

        void FrontierEvaluator::setupFromParamMap(Module::ParamMap *param_map) {
            SimulatedSensorEvaluator::setupFromParamMap(param_map);
            setParam<bool>(param_map, "accurate_frontiers", &p_accurate_frontiers_, false);
            setParam<bool>(param_map, "surface_frontiers", &p_surface_frontiers_, true);
            setParam<double>(param_map, "checking_distance", &p_checking_distance_, 1.0);

            // initialize neighbor offsets
            double vs = static_cast<double>(voxblox_ptr_->getEsdfMapPtr()->voxel_size());
            vs *= p_checking_distance_;
            if (!p_accurate_frontiers_) {
                c_neighbor_voxels_[0] = Eigen::Vector3d(vs, 0, 0);
                c_neighbor_voxels_[1] = Eigen::Vector3d(-vs, 0, 0);
                c_neighbor_voxels_[2] = Eigen::Vector3d(0, vs, 0);
                c_neighbor_voxels_[3] = Eigen::Vector3d(0, -vs, 0);
                c_neighbor_voxels_[4] = Eigen::Vector3d(0, 0, vs);
                c_neighbor_voxels_[5] = Eigen::Vector3d(0, 0, -vs);
            } else {
                c_neighbor_voxels_[0] = Eigen::Vector3d(vs, 0, 0);
                c_neighbor_voxels_[1] = Eigen::Vector3d(vs, vs, 0);
                c_neighbor_voxels_[2] = Eigen::Vector3d(vs, -vs, 0);
                c_neighbor_voxels_[3] = Eigen::Vector3d(vs, 0, vs);
                c_neighbor_voxels_[4] = Eigen::Vector3d(vs, vs, vs);
                c_neighbor_voxels_[5] = Eigen::Vector3d(vs, -vs, vs);
                c_neighbor_voxels_[6] = Eigen::Vector3d(vs, 0, -vs);
                c_neighbor_voxels_[7] = Eigen::Vector3d(vs, vs, -vs);
                c_neighbor_voxels_[8] = Eigen::Vector3d(vs, -vs, -vs);
                c_neighbor_voxels_[9] = Eigen::Vector3d(0, vs, 0);
                c_neighbor_voxels_[10] = Eigen::Vector3d(0, -vs, 0);
                c_neighbor_voxels_[11] = Eigen::Vector3d(0, 0, vs);
                c_neighbor_voxels_[12] = Eigen::Vector3d(0, vs, vs);
                c_neighbor_voxels_[13] = Eigen::Vector3d(0, -vs, vs);
                c_neighbor_voxels_[14] = Eigen::Vector3d(0, 0, -vs);
                c_neighbor_voxels_[15] = Eigen::Vector3d(0, vs, -vs);
                c_neighbor_voxels_[16] = Eigen::Vector3d(0, -vs, -vs);
                c_neighbor_voxels_[17] = Eigen::Vector3d(-vs, 0, 0);
                c_neighbor_voxels_[18] = Eigen::Vector3d(-vs, vs, 0);
                c_neighbor_voxels_[19] = Eigen::Vector3d(-vs, -vs, 0);
                c_neighbor_voxels_[20] = Eigen::Vector3d(-vs, 0, vs);
                c_neighbor_voxels_[21] = Eigen::Vector3d(-vs, vs, vs);
                c_neighbor_voxels_[22] = Eigen::Vector3d(-vs, -vs, vs);
                c_neighbor_voxels_[23] = Eigen::Vector3d(-vs, 0, -vs);
                c_neighbor_voxels_[24] = Eigen::Vector3d(-vs, vs, -vs);
                c_neighbor_voxels_[25] = Eigen::Vector3d(-vs, -vs, -vs);
            }
        }

        bool FrontierEvaluator::isFrontierVoxel(const Eigen::Vector3d &voxel) {
            voxblox::EsdfMap *esdf_map = voxblox_ptr_->getEsdfMapPtr().get();
            double distance;
            // Check all neighboring voxels
            if (!p_accurate_frontiers_) {
                for (int i = 0; i < 6; ++i) {
                    if (esdf_map->getDistanceAtPosition(voxel + c_neighbor_voxels_[i], &distance)) {
                        if (p_surface_frontiers_) {
                            return distance < c_voxel_size_;
                        } else {
                            return true;
                        }
                    }
                }
            } else {
                for (int i = 0; i < 26; ++i) {
                    if (esdf_map->getDistanceAtPosition(voxel + c_neighbor_voxels_[i], &distance)) {
                        if (p_surface_frontiers_) {
                            return distance < c_voxel_size_;
                        } else {
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        bool FrontierEvaluator::computeGainFromVisibleVoxels(TrajectorySegment *traj_in) {
            traj_in->gain = 0.0;
            if (!traj_in->info) {
                return false;
            }
            SimulatedSensorInfo *info = dynamic_cast<SimulatedSensorInfo *>(traj_in->info.get());

            // Remove observed voxels
            info->visible_voxels.erase(
                    std::remove_if(info->visible_voxels.begin(), info->visible_voxels.end(),
                                   [this](const Eigen::Vector3d &voxel) {
                                       return voxblox_ptr_->getEsdfMapPtr()->isObserved(voxel);
                                   }),
                    info->visible_voxels.end());

            // Check which voxels are frontier voxels
            for (int i = 0; i < info->visible_voxels.size(); ++i) {
                if (isFrontierVoxel(info->visible_voxels[i])) {
                    traj_in->gain += 1.0;
                }
            }
            return true;
        }

        void FrontierEvaluator::visualizeTrajectoryValue(visualization_msgs::MarkerArray *msg, const TrajectorySegment &trajectory) {
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
            SimulatedSensorInfo *info = dynamic_cast<SimulatedSensorInfo *>(trajectory.info.get());
            for (int i = 0; i < info->visible_voxels.size(); ++i) {
                if (isFrontierVoxel(info->visible_voxels[i])) {
                    geometry_msgs::Point point;
                    point.x = info->visible_voxels[i].x();
                    point.y = info->visible_voxels[i].y();
                    point.z = info->visible_voxels[i].z();
                    new_msg.points.push_back(point);
                }
            }
            msg->markers.push_back(new_msg);

            if (p_visualize_sensor_view_) {
                sensor_model_->visualizeSensorView(msg, trajectory);
            }
        }

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
