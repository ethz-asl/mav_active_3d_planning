#define _USE_MATH_DEFINES

#include "mav_active_3d_planning/trajectory_generator.h"
#include "mav_active_3d_planning/defaults.h"
#include "mav_active_3d_planning/libs/kdtree/kdtree.h"

#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/param.h>

#include <vector>
#include <random>
#include <cmath>

namespace mav_active_3d_planning {
    namespace trajectory_generators {

        class RRT : public TrajectoryGenerator {
        public:
            RRT(voxblox::EsdfServer *voxblox_ptr, std::string param_ns);

            // Overwrite virtual functions
            TrajectorySegment* selectSegment(TrajectorySegment &root);
            bool expandSegment(TrajectorySegment &target);

        protected:
            // parameters
            double p_velocity_;         // m/s
            double p_sampling_rate_;    // Hz
            double p_extension_range_;  // m (set 0.0 to ignore)
            bool p_use_spheric_sampling_;
            int p_maximum_tries_;       // 0 for inf

            // kdtree + bookkeeping
            kdtree* kdtree_;
            TrajectorySegment* previous_root_;
        };

        RRT::RRT(voxblox::EsdfServer *voxblox_ptr, std::string param_ns)
                : TrajectoryGenerator(voxblox_ptr, param_ns),
                  previous_root_(nullptr) {
            // params
            ros::param::param<double>(param_ns + "/velocity", p_velocity_, 0.5);
            ros::param::param<double>(param_ns + "/sampling_rate", p_sampling_rate_, 20.0);
            ros::param::param<double>(param_ns + "/extension_range", p_extension_range_, 1.0);
            ros::param::param<bool>(param_ns + "/use_spheric_sampling", p_use_spheric_sampling_, false);
            ros::param::param<int>(param_ns + "/maximum_tries", p_maximum_tries_, 0);

            kdtree_ = kd_create(3);     // the data in the kdtree is a pointer to its corresponding trajectorySegment
        }

        TrajectorySegment* RRT::selectSegment(TrajectorySegment &root){
            // If the root has changed, reset the kdtree and populate with the actual trajectory tree
            if (previous_root_ != &root){
                kd_clear(kdtree_);
                std::vector<TrajectorySegment*> current_tree;
                root.getTree(current_tree);
                Eigen::Vector3d end_point;
                for (int i = 0; i < current_tree.size(); ++i){
                    end_point = current_tree[i]->trajectory.back().position_W;
                    kd_insert3(kdtree_, end_point[0], end_point[1], end_point[2], current_tree[i]);
                }
                previous_root_ = &root;
            }
            // just return the root to expandSegment()
            return &root;
        }

        bool RRT::expandSegment(TrajectorySegment &target) {
            // sample candidate points
            bool goal_found = false;
            Eigen::Vector3d goal_pos;
            int counter = 0;
            while (!goal_found && counter <= p_maximum_tries_) {
                if (p_maximum_tries_ > 0) { counter++; }
                if (p_use_spheric_sampling_) {
                    // Bircher way (also assumes box atm, for unbiased sampling)
                    double radius = std::sqrt(std::pow(bounding_volume_.x_max - bounding_volume_.x_min, 2.0) +
                                              std::pow(bounding_volume_.y_max - bounding_volume_.y_min, 2.0) +
                                              std::pow(bounding_volume_.z_max - bounding_volume_.z_min, 2.0));
                    goal_pos = target.trajectory.back().position_W;
                    goal_pos[0] += 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
                    if (goal_pos[0] < bounding_volume_.x_min) {
                        continue;
                    } else if (goal_pos[0] > bounding_volume_.x_max) {
                        continue;
                    }
                    goal_pos[1] += 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
                    if (goal_pos[1] < bounding_volume_.y_min) {
                        continue;
                    } else if (goal_pos[1] > bounding_volume_.y_max) {
                        continue;
                    }
                    goal_pos[2] += 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
                    if (goal_pos[2] < bounding_volume_.z_min) {
                        continue;
                    } else if (goal_pos[2] > bounding_volume_.z_max) {
                        continue;
                    }
                } else {
                    // sample from bounding volume (assumes box atm)
                    goal_pos[0] = bounding_volume_.x_min +
                        (double) rand() / RAND_MAX * (bounding_volume_.x_max - bounding_volume_.x_min);
                    goal_pos[1] = bounding_volume_.y_min +
                        (double) rand() / RAND_MAX * (bounding_volume_.y_max - bounding_volume_.y_min);
                    goal_pos[2] = bounding_volume_.z_min +
                        (double) rand() / RAND_MAX * (bounding_volume_.z_max - bounding_volume_.z_min);
                }
                if (!checkTraversable(goal_pos)){
                    continue;
                }
                goal_found = true;
            }
            if (!goal_found){ return false; }

            // find closest point
            kdres * nearest = kd_nearest3(kdtree_, goal_pos[0], goal_pos[1], goal_pos[2]);
            if (kd_res_size(nearest) <= 0) {
                kd_res_free(nearest);
                return false;
            }
            TrajectorySegment* new_parent = (TrajectorySegment *) kd_res_item_data(nearest);
            kd_res_free(nearest);

            // Check max segment range
            Eigen::Vector3d start_pos = new_parent->trajectory.back().position_W;
            Eigen::Vector3d direction = goal_pos - start_pos;
            if (direction.norm() > p_extension_range_ && p_extension_range_ > 0.0){
                direction *= p_extension_range_ / direction.norm();
            }

            // try creating a linear trajectory and check for collision
            Eigen::Vector3d current_pos;
            int n_points = std::ceil(direction.norm() / (double) voxblox_ptr_->getEsdfMapPtr()->voxel_size());
            bool collided = false;
            for (int i = 0; i < n_points; ++i) {
                current_pos = start_pos + (double) i / (double) n_points * direction;
                if (!checkTraversable(current_pos)) {
                    collided = true;
                    break;
                }
            }
            if (collided) {
                return false;
            }

            // Build result
            TrajectorySegment *new_segment = new_parent->spawnChild();
            double goal_yaw = (double) rand() / (double) RAND_MAX * 2.0 * M_PI;
            n_points = std::ceil(direction.norm() / p_velocity_ * p_sampling_rate_);
            kd_insert3(kdtree_, goal_pos[0], goal_pos[1], goal_pos[2], new_segment);
            for (int i = 0; i < n_points; ++i) {
                mav_msgs::EigenTrajectoryPoint trajectory_point;
                trajectory_point.position_W = start_pos + (double) i / (double) n_points * direction;
                trajectory_point.setFromYaw(goal_yaw);
                trajectory_point.time_from_start_ns = static_cast<int64_t>((double) i / p_sampling_rate_ * 1.0e9);
                new_segment->trajectory.push_back(trajectory_point);
            }
            return true;
        }

    } // namespace trajectory_generators
}  // namespace mav_active_3d_planning