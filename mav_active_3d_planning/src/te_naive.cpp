#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/trajectory_segment.h"
#include "../include/mav_active_3d_planning/trajectory_segment.h"

#include <mav_msgs/eigen_mav_msgs.h>
#include <voxblox_ros/esdf_server.h>

#include <vector>
#include <math.h>
#include <chrono>

namespace mav_active_3d_planning {

    class TENaive: public TrajectoryEvaluator {
    public:
        TENaive(voxblox::EsdfServer *voxblox_Ptr) : TrajectoryEvaluator(voxblox_Ptr) {}

        // Overwrite virtual functions
        bool computeValue(TrajectorySegment &traj_in);
        bool computeCost(TrajectorySegment &traj_in);
        bool computeGain(TrajectorySegment &traj_in);
        int selectNextBest(TrajectorySegment &traj_in);
        bool setParamsFromRos(const ros::NodeHandle &nh);

        // member functions
        std::vector<voxblox::GlobalIndex> getVisibleVoxels(Eigen::Vector3d position, Eigen::Quaterniond orientation);
        double evaluateSingle(TrajectorySegment *traj_in);

    protected:
        // parameters
        double p_ray_length_;
        double p_field_of_view_x_;
        double p_field_of_view_y_;
        double p_focal_length_;
        double p_sampling_time_;
        int p_resolution_x_;
        int p_resolution_y_;
        double p_ray_step_;
        double p_cost_weight_;

        // constants
        voxblox::FloatingPoint c_voxel_size_;
        voxblox::FloatingPoint c_block_size_;
    };

    bool TENaive::setParamsFromRos(const ros::NodeHandle &nh) {
        c_voxel_size_ = voxblox_Ptr_->getEsdfMapPtr()->voxel_size();
        c_block_size_ = voxblox_Ptr_->getEsdfMapPtr()->block_size();

        nh.param("TE_ray_length", p_ray_length_, 5.0);
        nh.param("TE_focal_length", p_focal_length_, 320.0);
        nh.param("TE_sampling_time", p_sampling_time_, 10.0);   // Will always run once on initial point
        nh.param("TE_resolution_x", p_resolution_x_, 640);
        nh.param("TE_resolution_y", p_resolution_y_, 480);
        nh.param("TE_ray_step", p_ray_step_, static_cast<double>(c_voxel_size_));
        nh.param("TE_cost_weight", p_cost_weight_, 1.0e-6);

        p_field_of_view_x_ = atan(2.0 * p_focal_length_ / p_resolution_x_);
        p_field_of_view_y_ = atan(2.0 * p_focal_length_ / p_resolution_y_);
        return true;
    }

    bool TENaive::computeGain(TrajectorySegment &traj_in) {
        // Sample in time
        int64_t current_time = 0;
        for(int i = 0; i < traj_in.trajectory.size(); i++) {
            if (traj_in.trajectory[i].time_from_start_ns >= current_time) {
                current_time += static_cast<int64_t>(p_sampling_time_ * 1.0e9);
                std::vector <voxblox::GlobalIndex> new_indices = getVisibleVoxels(traj_in.trajectory[i].position_W,
                                                                                  traj_in.trajectory[i].orientation_W_B);
                if(false){  // This takes a lot of time and is not yet necessary
                    // Remove previously seen voxels
                    TrajectorySegment* previous = traj_in.parent;
                    while (previous){
                        std::vector <voxblox::GlobalIndex> old_indices = previous->info;
                        new_indices.erase(
                                std::remove_if(
                                        new_indices.begin(),
                                        new_indices.end(),
                                        [&old_indices](const voxblox::GlobalIndex& global_index)
                                        {
                                            auto it = std::find(old_indices.begin(), old_indices.end(), global_index);
                                            return (it != old_indices.end());
                                        }),
                                new_indices.end());
                        previous = previous->parent;
                    }
                }
                // Store info
                traj_in.gain = (double)new_indices.size();
                traj_in.info = new_indices;
            }
        }
        return true;
    }

    std::vector<voxblox::GlobalIndex> TENaive::getVisibleVoxels(Eigen::Vector3d position, Eigen::Quaterniond orientation) {
        // Returns the global indices of all visible, newly discovered voxels for a simple camera model pointing in x-direction of position
        std::vector <voxblox::GlobalIndex> result;

        // Downsample to voxel size resolution at max range
        int res_x = ceil(2.0*p_ray_length_*sin(p_field_of_view_x_/2.0)/c_voxel_size_);
        int res_y = ceil(2.0*p_ray_length_*sin(p_field_of_view_y_/2.0)/c_voxel_size_);

        // Naive ray-casting
        Eigen::Vector3d unit_direction = orientation * Eigen::Vector3d(1, 0, 0);
        for (int i = 0; i < res_x; ++i) {
            Eigen::Vector3d x_direction =  Eigen::AngleAxis<double>(p_field_of_view_x_ * ((double) i / (double) res_x - 0.5), Eigen::Vector3d(0, 0, 1)) * unit_direction;
            for (int j = 0; j < res_y; ++j) {
                Eigen::Vector3d direction = Eigen::AngleAxis<double>(
                        p_field_of_view_y_ * ((double) j / (double) res_y - 0.5),
                        Eigen::Vector3d(0, 1, 0)) * x_direction;
                double distance = 0.0;
                while (distance < p_ray_length_) {
                    Eigen::Vector3d current_position = position + distance * direction;
                    distance += p_ray_step_;
                    // Check voxel occupied
                    double map_distance = 0.0;
                    if (voxblox_Ptr_->getEsdfMapPtr()->getDistanceAtPosition(current_position, &map_distance)) {
                        if (map_distance < 0.0) {
                            break;
                        }
                        if (voxblox_Ptr_->getEsdfMapPtr()->isObserved(current_position)){
                            continue;
                        }
                    }

                    // Add new virtual voxel
                    voxblox::BlockIndex block_id = voxblox_Ptr_->getEsdfMapPtr()->getEsdfLayerPtr()->computeBlockIndexFromCoordinates(
                            current_position.cast<voxblox::FloatingPoint>());
                    voxblox::VoxelIndex voxel_id = voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
                            current_position.cast<voxblox::FloatingPoint>() -
                            voxblox::getOriginPointFromGridIndex(block_id, c_block_size_),
                            1.0 / c_voxel_size_);
                    voxblox::GlobalIndex global_id = voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(block_id,
                            voxel_id,  (int) round(c_block_size_/c_voxel_size_));

                    result.push_back(global_id);
                }
            }
        }
        result.erase( std::unique( result.begin(), result.end()), result.end() );
        return result;
    }

    bool TENaive::computeCost(TrajectorySegment &traj_in){
        // Simple cost: time to execute trajectory (ns)
        traj_in.cost = (double)traj_in.trajectory.back().time_from_start_ns - traj_in.trajectory.front().time_from_start_ns;
        return true;
    }

    bool TENaive::computeValue(TrajectorySegment &traj_in){
        // Accumulated gain and cost (all after root segment, weighted)
        double cost= traj_in.cost;
        double gain =traj_in.gain;
        TrajectorySegment* current = &traj_in;
        while (current->parent){
            cost += current->cost;
            gain += current->gain;
            current = current->parent;
        }
        traj_in.value = gain - p_cost_weight_ * cost;
        return true;
    }

    int TENaive::selectNextBest(TrajectorySegment &traj_in) {
        // find the child segment with highest value anywhere subsequently
        if (traj_in.children.size() == 1) {
            return 0;
        }
        int current_index = 0;
        double current_value, best_value = evaluateSingle(traj_in.children[0].get());
        for (int i = 1; i < traj_in.children.size(); ++i) {
            current_value = evaluateSingle(traj_in.children[i].get());
            if (current_value > best_value){
                best_value = current_value;
                current_index = i;
            }
        }
        return current_index;
    }

    double TENaive::evaluateSingle(TrajectorySegment* traj_in){
        // Return highest segment value from children or self
        if (traj_in->children.empty()){
            return traj_in->value;
        }
        double highest_value = traj_in->value;
        for (int i=0; i < traj_in->children.size(); ++i) {
            highest_value = std::max(highest_value, evaluateSingle(traj_in->children[i].get()));
        }
        return highest_value;
    }

}  // namespace mav_active_3d_planning