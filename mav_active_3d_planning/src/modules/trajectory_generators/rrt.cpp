#define _USE_MATH_DEFINES

#include "mav_active_3d_planning/modules/trajectory_generators/rrt.h"
#include "mav_active_3d_planning/defaults.h"

#include <mav_msgs/eigen_mav_msgs.h>

#include <vector>
#include <random>
#include <cmath>
#include <memory>

namespace mav_active_3d_planning {
    namespace trajectory_generators {

        void RRT::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<bool>(param_map, "crop_segments", &p_crop_segments_, false);
            setParam<double>(param_map, "crop_margin", &p_crop_margin_, 0.1);
            setParam<double>(param_map, "crop_min_length", &p_crop_min_length_, 0.2);
            setParam<double>(param_map, "sampling_rate", &p_sampling_rate_, 20.0);
            setParam<double>(param_map, "max_extension_range", &p_max_extension_range_, 1.0);
            setParam<bool>(param_map, "use_spheric_sampling", &p_use_spheric_sampling_, false);
            setParam<int>(param_map, "maximum_tries", &p_maximum_tries_, 1000);

            // setup parent
            TrajectoryGenerator::setupFromParamMap(param_map);
        }

        bool RRT::checkParamsValid(std::string *error_message) {
            if (p_sampling_rate_ <= 0.0) {
                *error_message = "sampling_rate expected > 0";
                return false;
            }
            if (p_crop_margin_ <= 0.0) {
                *error_message = "crop_margin expected > 0";
                return false;
            }
            return TrajectoryGenerator::checkParamsValid(error_message);
        }

        bool RRT::selectSegment(TrajectorySegment **result, TrajectorySegment *root) {
            // If the root has changed, reset the kdtree and populate with the current trajectory tree
            if (previous_root_ != root) {
                std::vector < TrajectorySegment * > currrent_tree;
                root->getTree(&currrent_tree);
                tree_data_.clear();
                for (int i = 0; i < currrent_tree.size(); ++i) {
                    tree_data_.addSegment(currrent_tree[i]);
                }
                kdtree_ = std::unique_ptr<KDTree>(new KDTree(3, tree_data_));
                kdtree_->addPoints(0, tree_data_.points.size() - 1);
                previous_root_ = root;
            }

            // sample candidate points
            bool goal_found = false;
            Eigen::Vector3d goal_pos;
            int counter = 0;
            while (!goal_found && counter <= p_maximum_tries_) {
                if (p_maximum_tries_ > 0) {
                    counter++;
                }
                goal_pos = root->trajectory.back().position_W;
                sample_goal(&goal_pos);
                if (p_crop_segments_ || checkTraversable(goal_pos)) {
                    goal_found = true;
                }

            }
            if (!goal_found) {
                *result = nullptr;
                return false;
            }

            // find closest point in kdtree
            double query_pt[3] = {goal_pos.x(), goal_pos.y(), goal_pos.z()};
            std::size_t ret_index;
            double out_dist_sqr;
            nanoflann::KNNResultSet<double> resultSet(1);
            resultSet.init(&ret_index, &out_dist_sqr);
            if (!kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10))) {
                *result = nullptr;
                return false;
            }

            // Valid target found
            goal_pos_ = goal_pos;
            *result = tree_data_.data[ret_index];
            return true;
        }

        bool RRT::expandSegment(TrajectorySegment *target, std::vector<TrajectorySegment *> *new_segments) {
            if (!target) {
                // Segment selection failed
                return false;
            }

            // Check max segment range
            Eigen::Vector3d start_pos = target->trajectory.back().position_W;
            Eigen::Vector3d direction = goal_pos_ - start_pos;
            if (p_max_extension_range_ > 0.0 && direction.norm() > p_max_extension_range_) {
                direction *= p_max_extension_range_ / direction.norm();
            }
            if (p_crop_segments_) {
                // if the full length cannot be reached, crop it
                int n_points = std::ceil(direction.norm() / (double) voxblox_ptr_->getEsdfMapPtr()->voxel_size());
                for (int i = 0; i < n_points; ++i) {
                    if (!checkTraversable(start_pos + (double) i / (double) n_points * direction)) {
                        double length = direction.norm() * (double) (i - 1) / (double) n_points - p_crop_margin_;
                        if (length <= p_crop_min_length_){
                            return false;
                        }
                        direction *= length / direction.norm();
                        break;
                    }
                }
            }

            // try creating a trajectory
            mav_msgs::EigenTrajectoryPointVector trajectory;
            mav_msgs::EigenTrajectoryPoint start_point = target->trajectory.back();
            mav_msgs::EigenTrajectoryPoint goal_point;
            goal_point.position_W = start_pos + direction;
            goal_point.setFromYaw((double) rand() / (double) RAND_MAX * 2.0 * M_PI);    // random orientation
            if (!connect_poses(start_point, goal_point, &trajectory)) {
                return false;
            }

            // Build result and add it to the kdtree
            TrajectorySegment *new_segment = target->spawnChild();
            new_segment->trajectory = trajectory;
            new_segments->push_back(new_segment);
            tree_data_.addSegment(new_segment);
            kdtree_->addPoints(tree_data_.points.size() - 1, tree_data_.points.size() - 1);
            return true;
        }

        bool RRT::sample_goal(Eigen::Vector3d *goal_pos) {
            if (p_use_spheric_sampling_) {
                // Bircher way (also assumes box atm, for unbiased sampling)
                double radius = std::sqrt(std::pow(bounding_volume_.x_max - bounding_volume_.x_min, 2.0) +
                                          std::pow(bounding_volume_.y_max - bounding_volume_.y_min, 2.0) +
                                          std::pow(bounding_volume_.z_max - bounding_volume_.z_min, 2.0));
                for (int i = 0; i < 3; i++) {
                    (*goal_pos)[i] += 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
                }
                return true;
            } else {
                // sample from bounding volume (assumes box atm)
                (*goal_pos)[0] = bounding_volume_.x_min +
                                 (double) rand() / RAND_MAX * (bounding_volume_.x_max - bounding_volume_.x_min);
                (*goal_pos)[1] = bounding_volume_.y_min +
                                 (double) rand() / RAND_MAX * (bounding_volume_.y_max - bounding_volume_.y_min);
                (*goal_pos)[2] = bounding_volume_.z_min +
                                 (double) rand() / RAND_MAX * (bounding_volume_.z_max - bounding_volume_.z_min);
                return true;
            }
        }

        bool RRT::connect_poses(const mav_msgs::EigenTrajectoryPoint &start, const mav_msgs::EigenTrajectoryPoint &goal,
                                mav_msgs::EigenTrajectoryPointVector *result) {
            // try creating a linear trajectory and check for collision
            Eigen::Vector3d start_pos = start.position_W;
            Eigen::Vector3d direction = goal.position_W - start_pos;
            int n_points = std::ceil(direction.norm() / (double) voxblox_ptr_->getEsdfMapPtr()->voxel_size());
            for (int i = 0; i < n_points; ++i) {
                if (!checkTraversable(start_pos + (double) i / (double) n_points * direction)) {
                    return false;
                }
            }

            // Build trajectory
            n_points = std::ceil(direction.norm() / system_constraints_.v_max * p_sampling_rate_);
            for (int i = 0; i < n_points; ++i) {
                mav_msgs::EigenTrajectoryPoint trajectory_point;
                trajectory_point.position_W = start_pos + (double) i / (double) n_points * direction;
                trajectory_point.setFromYaw(goal.getYaw());
                trajectory_point.time_from_start_ns = static_cast<int64_t>((double) i / p_sampling_rate_ * 1.0e9);
                result->push_back(trajectory_point);
            }
            return true;
        }

        void RRT::TreeData::clear() {
            points.clear();
            data.clear();
        }

        void RRT::TreeData::addSegment(TrajectorySegment *to_add) {
            points.push_back(to_add->trajectory.back().position_W);
            data.push_back(to_add);
        }

    } // namespace trajectory_generators
}  // namespace mav_active_3d_planning