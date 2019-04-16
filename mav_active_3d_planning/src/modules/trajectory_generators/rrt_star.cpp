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
        ModuleFactory::Registration <RRTStar> RRTStar::registration("RRTStar");

        void RRTStar::setupFromParamMap(Module::ParamMap *param_map) {
            RRT::setupFromParamMap(param_map);
            setParam<bool>(param_map, "rewire_root", &p_rewire_root_, false);
            setParam<bool>(param_map, "rewire_intermediate", &p_rewire_intermediate_, false);
            setParam<double>(param_map, "max_rewire_range", &p_max_rewire_range_, p_max_extension_range_ + 0.2);
            setParam<int>(param_map, "n_neighbors", &p_n_neighbors_, 10);
            c_rewire_range_square_ = p_max_rewire_range_*p_max_rewire_range_;
            ModuleFactory::Instance()->registerLinkableModule("RRTStarGenerator", this);
            planner_node_ = dynamic_cast<PlannerNode *>(ModuleFactory::Instance()->readLinkableModule("PlannerNode"));
        }

        bool RRTStar::checkParamsValid(std::string *error_message) {
            if (p_max_rewire_range_ <= 0.0) {
                *error_message = "rewire_range expected > 0";
                return false;
            }
            return RRT::checkParamsValid(error_message);
        }

        bool RRTStar::expandSegment(TrajectorySegment *target, std::vector<TrajectorySegment *> *new_segments) {
            tree_is_reset_ = true;  // select was called earlier, resetting the tree on new root segment
            if (!target) {
                // Segment selection failed
                return false;
            }

            // Check max segment range and cropping
            if (!adjustGoalPosition(target->trajectory.back().position_W, &goal_pos_)) {
                return false;
            }

            // Find nearby parent candidates
            std::vector < TrajectorySegment * > candidate_parents;
            if (!findNearbyCandidates(goal_pos_, &candidate_parents)) {
                // No candidates found
                return false;
            }

            // Compute the gain of the new point (evaluation must be single point!)
            TrajectorySegment *new_segment = new TrajectorySegment();
            mav_msgs::EigenTrajectoryPoint goal_point;
            goal_point.position_W = goal_pos_;
            goal_point.setFromYaw((double) rand() / (double) RAND_MAX * 2.0 * M_PI);    // random orientation
            new_segment->trajectory.push_back(goal_point);
            new_segment->parent = nullptr;
            planner_node_->trajectory_evaluator_->computeGain(new_segment);

            // Create the trajectory from the best parent and attach the segment to it
            if (!rewireToBestParent(new_segment, candidate_parents)) {
                // no possible trajectory found
                delete new_segment;
                return false;
            }

            // Rewire existing segments within range to the new segment, if this increases their value
            if (p_rewire_intermediate_) {
                TrajectorySegment *current = new_segment;
                while (current) {
                    // the connection of the new segment to the root cannot be rewired (loops!)
                    candidate_parents.erase(std::remove(candidate_parents.begin(), candidate_parents.end(), current),
                                            candidate_parents.end());
                    current = current->parent;
                }
                std::vector < TrajectorySegment * > new_parent = {new_segment};
                for (int i = 0; i < candidate_parents.size(); ++i) {
                    rewireToBestParent(candidate_parents[i], new_parent);
                }
            }

            // Add to the kdtree
            tree_data_.addSegment(new_segment);
            kdtree_->addPoints(tree_data_.points.size() - 1, tree_data_.points.size() - 1);
            return true;
        }

        bool RRTStar::rewireRoot(TrajectorySegment *root, int *next_segment) {
            if (!p_rewire_root_) {
                return true;
            }
            if (!tree_is_reset_) {
                // Force reset (dangling pointers!)
                std::vector < TrajectorySegment * > currrent_tree;
                root->getTree(&currrent_tree);
                tree_data_.clear();
                for (int i = 0; i < currrent_tree.size(); ++i) {
                    tree_data_.addSegment(currrent_tree[i]);
                }
                kdtree_ = std::unique_ptr<KDTree>(new KDTree(3, tree_data_));
                kdtree_->addPoints(0, tree_data_.points.size() - 1);
            }
            tree_is_reset_ = false;
            // Try rewiring non-next segments (to keept their branches alive)
            TrajectorySegment *next_root = root->children[*next_segment].get();
            std::vector < TrajectorySegment * > to_rewire;
            root->getChildren(&to_rewire);
            to_rewire.erase(std::remove(to_rewire.begin(), to_rewire.end(), next_root), to_rewire.end());
            for (int i = 0; i < to_rewire.size(); ++i) {
                std::vector < TrajectorySegment * > candidate_parents;
                if (!findNearbyCandidates(to_rewire[i]->trajectory.back().position_W, &candidate_parents)) {
                    continue;
                }
                // remove all candidates that are still connected to the root
                std::vector < TrajectorySegment * > safe_candidates;
                TrajectorySegment *current;
                for (int j = 0; j < candidate_parents.size(); ++j) {
                    current = candidate_parents[j];
                    while (current) {
                        if (current == next_root) {
                            safe_candidates.push_back(candidate_parents[j]);
                            break;
                        }
                        current = current->parent;
                    }
                }
                if (safe_candidates.empty()) { continue; }
                rewireToBestParent(to_rewire[i], safe_candidates);
            }

            // Adjust the next best value
            for (int i = 0; i < root->children.size(); ++i) {
                if (root->children[i].get() == next_root) {
                    *next_segment = i;
                    break;
                }
            }
            return true;
        }

        bool
        RRTStar::findNearbyCandidates(const Eigen::Vector3d &target_point, std::vector<TrajectorySegment *> *result) {
            // Also tried radius search here but that stuff blows for some reason...
            double query_pt[3] = {target_point.x(), target_point.y(), target_point.z()};
            std::size_t ret_index[p_n_neighbors_];
            double out_dist[p_n_neighbors_];
            nanoflann::KNNResultSet<double> resultSet(p_n_neighbors_);
            resultSet.init(ret_index, out_dist);
            kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));
            bool candidate_found = false;
            for (int i = 0; i < resultSet.size(); ++i) {
                if (out_dist[i] <= c_rewire_range_square_) {
                    candidate_found = true;
                    result->push_back(tree_data_.data[ret_index[i]]);
                }
            }
            return candidate_found;
        }

        bool RRTStar::rewireToBestParent(TrajectorySegment *segment,
                                         const std::vector<TrajectorySegment *> &candidates) {
            // Evaluate all candidate parents and store the best one in the segment
            // Goal is the end point of the segment
            mav_msgs::EigenTrajectoryPoint goal_point = segment->trajectory.back();

            // store the initial segment
            TrajectorySegment best_segment = segment->shallowCopy();
            TrajectorySegment *initial_parent = segment->parent;

            // Find best segment
            for (int i = 0; i < candidates.size(); ++i) {
                segment->trajectory.clear();
                segment->parent = candidates[i];
                if (connect_poses(candidates[i]->trajectory.back(), goal_point, &(segment->trajectory))) {
                    // Feasible connection: evaluate the trajectory
                    planner_node_->trajectory_evaluator_->computeCost(segment);
                    planner_node_->trajectory_evaluator_->computeValue(segment);
                    if (best_segment.parent == nullptr || segment->value > best_segment.value) {
                        best_segment = segment->shallowCopy();
                    }
                }
            }
            if (best_segment.parent == nullptr) {
                // No connection found and no previous trajectory
                return false;
            } else {
                // Apply best segment and rewire
                segment->cost = best_segment.cost;
                segment->value = best_segment.value;
                segment->parent = best_segment.parent;
                segment->trajectory = best_segment.trajectory;
                if (segment->parent == initial_parent) {
                    // Back to old parent
                    return true;
                } else if (initial_parent == nullptr) {
                    // Found new parent
                    segment->parent->children.push_back(std::unique_ptr<TrajectorySegment>(segment));
                    return true;
                } else {
                    for (int i = 0; i < initial_parent->children.size(); ++i) {
                        if (initial_parent->children[i].get() == segment) {
                            // Move from existing parent
                            segment->parent->children.push_back(std::move(initial_parent->children[i]));
                            initial_parent->children.erase(initial_parent->children.begin() + i);
                            return true;
                        }
                    }
                    // Rewiring failed (should not happen by construction)
                    return false;
                }
            }
        }

    } // namespace trajectory_generators

    namespace trajectory_evaluators {

        // RRTStarEvaluatorAdapter (just delegate everything, call rewire on select best)
        ModuleFactory::Registration <RRTStarEvaluatorAdapter> RRTStarEvaluatorAdapter::registration(
                "RRTStarEvaluatorAdapter");

        bool RRTStarEvaluatorAdapter::computeGain(TrajectorySegment *traj_in) {
            return following_evaluator_->computeGain(traj_in);
        }

        bool RRTStarEvaluatorAdapter::computeCost(TrajectorySegment *traj_in) {
            return following_evaluator_->computeCost(traj_in);
        }

        bool RRTStarEvaluatorAdapter::computeValue(TrajectorySegment *traj_in) {
            return following_evaluator_->computeValue(traj_in);
        }

        int RRTStarEvaluatorAdapter::selectNextBest(TrajectorySegment *traj_in) {
            int next = following_evaluator_->selectNextBest(traj_in);
            generator_->rewireRoot(traj_in, &next);
            return next;
        }

        bool RRTStarEvaluatorAdapter::updateSegments(TrajectorySegment *root) {
            return following_evaluator_->updateSegments(root);
        }

        void RRTStarEvaluatorAdapter::visualizeTrajectoryValue(visualization_msgs::MarkerArray *msg,
                                                               const TrajectorySegment &trajectory) {
            following_evaluator_->visualizeTrajectoryValue(msg, trajectory);
        }

        void RRTStarEvaluatorAdapter::setupFromParamMap(Module::ParamMap *param_map) {
            generator_ = dynamic_cast<mav_active_3d_planning::trajectory_generators::RRTStar *>(
                    ModuleFactory::Instance()->readLinkableModule("RRTStarGenerator"));
            // Create following evaluator
            std::string args;   // default args extends the parent namespace
            std::string param_ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "following_evaluator_args", &args,
                                  param_ns + "/following_evaluator");
            following_evaluator_ = ModuleFactory::Instance()->createModule<TrajectoryEvaluator>(args, verbose_modules_);

            // setup parent
            TrajectoryEvaluator::setupFromParamMap(param_map);
        }

    } //namespace trajectory_evaluators
}  // namespace mav_active_3d_planning