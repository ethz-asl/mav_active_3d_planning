#define _USE_MATH_DEFINES

#include "mav_active_3d_planning/modules/trajectory_generators/rrt_star.h"
#include "mav_active_3d_planning/defaults.h"

#include <mav_msgs/eigen_mav_msgs.h>

#include <vector>
#include <algorithm>
#include <random>
#include <cmath>

namespace mav_active_3d_planning {
    namespace trajectory_generators {

        // RRTStar
        void RRTStar::setupFromParamMap(Module::ParamMap *param_map) {
            RRT::setupFromParamMap(param_map);
            setParam<bool>(param_map, "rewire_root", &p_rewire_root_, false);
            setParam<double>(param_map, "rewire_range", &p_rewire_range_, p_max_extension_range_);
        }

        bool RRTStar::checkParamsValid(std::string *error_message) {
            if (p_rewire_range_ <= 0.0) {
                *error_message = "rewire_range expected > 0";
                return false;
            }
            return RRT::checkParamsValid(error_message);
        }

        bool RRTStar::expandSegment(TrajectorySegment *target, std::vector<TrajectorySegment *> *new_segments) {
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
                        if (length <= p_crop_min_length_) {
                            return false;
                        }
                        direction *= length / direction.norm();
                        break;
                    }
                }
            }

            // Compute the gain of the new point (evaluation must be single point!)
            TrajectorySegment *new_segment = new TrajectorySegment();
            mav_msgs::EigenTrajectoryPoint goal_point;
            goal_point.position_W = start_pos + direction;
            goal_point.setFromYaw((double) rand() / (double) RAND_MAX * 2.0 * M_PI);    // random orientation
            new_segment->trajectory.push_back(goal_point);
            parent_->trajectory_evaluator_->computeGain(new_segment);

            // Find nearby parent candidates
            std::vector <std::pair<std::size_t, double>> indices_dists;
            double query_pt[3] = {goal_pos.x(), goal_pos.y(), goal_pos.z()};
            nanoflann::RadiusResultSet<double, std::size_t> result_set(p_rewire_range_, indices_dists);
            kdtree_->findNeighbors(result_set, query_pt, nanoflann::SearchParams());
            if (indices_dists.empty()) {
                // no neighbors found
                delete new_segment;
                return false;
            }

            // Build best segment
            std::vector < TrajectorySegment * > candidate_parents;
            for (int i = 0; i < indices_dists.size(); ++i) {
                candidate_parents.push_back(tree_data_.data[indices_dists[i].first);
            }
            if (!findBestParentTrajectory(new_segment, candidate_parents)) {
                // no possible trajectory found
                delete new_segment;
                return false;
            }

            // Rewire existing segments within range to the new segment, if this increases their value
            TrajectorySegment *current = new_segment->parent;
            while (current) {
                // the connection of the new segment to the root cannot be rewired (loops!)
                candidate_parents.erase(std::remove(candidate_parents.begin(), candidate_parents.end(), current),
                                        candidate_parents.end());
                current = current->parent;
            }
            for (int i = 0; i < candidate_parents.size(); ++i) {

            }


            // Build result and add it to the kdtree
            TrajectorySegment *new_segment = target->spawnChild();
            new_segment->trajectory = trajectory;
            new_segments->push_back(new_segment);
            tree_data_.addSegment(new_segment);
            kdtree_->addPoints(tree_data_.points.size() - 1, tree_data_.points.size() - 1);
            return true;
        }

        bool findBestParentTrajectory(TrajectorySegment *segment, const std::vector<TrajectorySegment *> &candidates) {
            // Evaluate all candidate parents and store the best one in the segment
            // Goal is the end of the inital segment
            mav_msgs::EigenTrajectoryPoint goal_point = segment->trajectory.back();
            bool parent_found = false;
            mav_msgs::EigenTrajectoryPointVector best_trajectory;
            double best_value;
            double best_cost;
            TrajectorySegment *best_parent;
            for (int i = 0; i < candidates.size(); ++i) {
                mav_msgs::EigenTrajectoryPointVector trajectory;
                if (connect_poses(candidates[i]->trajectory.back(), goal_point, &trajectory)) {
                    // evaluate the trajectory (these ops should not touch the gain or info)
                    segment->trajectory = trajectory;
                    segment->parent = candidate_parent;
                    parent_->trajectory_evaluator_->computeCost(segment);
                    parent_->trajectory_evaluator_->computeValue(segment);
                    if (!parent_found) {
                        parent_found = true;
                        best_value = segment->value;
                        best_cost = segment->cost;
                        best_parent = candidates[i];
                        best_trajectory = trajectory;
                    } else if (new_segment->value > best_value) {
                        best_value = segment->value;
                        best_cost = segment->cost;
                        best_parent = candidates[i];
                        best_trajectory = trajectory;
                    }
                }
            }
            if (!parent_found) {
                return false;
            } else {
                segment->cost = best_cost;
                segment->value = best_value;
                segment->parent = best_parent;
                segment->trajectory = best_trajectory;
                return true;
            }
        }

    } // namespace trajectory_generators

    namespace trajectory_evaluators {

        // RRTStarEvaluatorAdapter (just delegate everything, call rewire on select best)
        bool RRTStarEvaluatorAdapter::computeGain(TrajectorySegment *traj_in) {
            return following_evaluator_->computeGain(traj_in);
        }

        bool RRTStarEvaluatorAdapter::computeCost(TrajectorySegment *traj_in) {
            return following_evaluator_->computeCost(traj_in);
        }

        bool RRTStarEvaluatorAdapter::computeValue(TrajectorySegment *traj_in) {
            return following_evaluator_->computeValue(traj_in);
        }

        int RRTStarEvaluatorAdapter::selectNextBest(const TrajectorySegment &traj_in) {
            dynamic_cast<RRTStar *>(parent_->trajectory_generator_.get())->rewireRoot();
            return following_evaluator_->selectNextBest(traj_in);
        }

        bool RRTStarEvaluatorAdapter::updateSegments(TrajectorySegment *root) {
            return following_evaluator_->updateSegments(root);
        }

        void RRTStarEvaluatorAdapter::visualizeTrajectoryValue(visualization_msgs::MarkerArray *msg,
                                                               const TrajectorySegment &trajectory) {
            following_evaluator_->visualizeTrajectoryValue(msg, trajectory);
        }

        void RRTStarEvaluatorAdapter::setupFromParamMap(Module::ParamMap *param_map) {
            // Create following evaluator
            std::string args;   // default args extends the parent namespace
            std::string param_ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "following_evaluator_args", &args,
                                  param_ns + "/following_evaluator");
            following_evaluator_ = ModuleFactory::Instance()->createTrajectoryEvaluator(args, parent_, voxblox_ptr_,
                                                                                        verbose_modules_);

            // setup parent
            TrajectoryEvaluator::setupFromParamMap(param_map);
        }

    } //namespace trajectory_evaluators
}  // namespace mav_active_3d_planning