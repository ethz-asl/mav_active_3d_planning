#define _USE_MATH_DEFINES

#include "active_3d_planning_core/module/trajectory_generator/rrt_star.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "active_3d_planning_core/data/trajectory.h"
#include "active_3d_planning_core/module/module_factory.h"
#include "active_3d_planning_core/planner/planner_I.h"

namespace active_3d_planning {
namespace trajectory_generator {

// RRTStar
ModuleFactoryRegistry::Registration<RRTStar> RRTStar::registration("RRTStar");

RRTStar::RRTStar(PlannerI& planner) : RRT(planner) {}

void RRTStar::setupFromParamMap(Module::ParamMap* param_map) {
  RRT::setupFromParamMap(param_map);
  setParam<bool>(param_map, "rewire_root", &p_rewire_root_, true);
  setParam<bool>(param_map, "rewire_intermediate", &p_rewire_intermediate_,
                 true);
  setParam<bool>(param_map, "rewire_update", &p_rewire_update_, true);
  setParam<bool>(param_map, "update_subsequent", &p_update_subsequent_, true);
  setParam<bool>(param_map, "reinsert_root", &p_reinsert_root_, true);
  setParam<double>(param_map, "max_rewire_range", &p_max_rewire_range_,
                   p_max_extension_range_ + 0.1);
  setParam<double>(param_map, "max_density_range", &p_max_density_range_, 0.0);
  setParam<int>(param_map, "n_neighbors", &p_n_neighbors_, 10);
  planner_.getFactory().registerLinkableModule("RRTStarGenerator", this);
}

bool RRTStar::checkParamsValid(std::string* error_message) {
  if (p_max_rewire_range_ <= 0.0) {
    *error_message = "rewire_range expected > 0";
    return false;
  }
  return RRT::checkParamsValid(error_message);
}

bool RRTStar::selectSegment(TrajectorySegment** result,
                            TrajectorySegment* root) {
  // If the root has changed, reset the kdtree and populate with the current
  // trajectory tree
  if (previous_root_ != root) {
    resetTree(root);
    previous_root_ = root;
    if (p_rewire_update_) {
      rewireIntermediate(root);
    }
    if (p_sampling_mode_ == "semilocal") {
      // Check whether the minimum number of local points is achieved and store
      // how many to go
      double query_pt[3] = {root->trajectory.back().position_W.x(),
                            root->trajectory.back().position_W.y(),
                            root->trajectory.back().position_W.z()};
      std::size_t ret_index[p_semilocal_count_];  // NOLINT
      double out_dist[p_semilocal_count_];        // NOLINT
      nanoflann::KNNResultSet<double> resultSet(p_semilocal_count_);
      resultSet.init(ret_index, out_dist);
      kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));
      semilocal_count_ = p_semilocal_count_;
      for (int i = 0; i < resultSet.size(); ++i) {
        if (out_dist[p_semilocal_count_ - 1] <=
            p_semilocal_radius_max_ * p_semilocal_radius_max_) {
          semilocal_count_ -= 1;
        }
      }
    }
  }
  return RRT::selectSegment(result, root);
}

bool RRTStar::expandSegment(TrajectorySegment* target,
                            std::vector<TrajectorySegment*>* new_segments) {
  tree_is_reset_ = true;  // select was called earlier, resetting the tree on
                          // new root segment
  if (!target) {
    // Segment selection failed
    return false;
  }

  // Check max segment range and cropping
  if (!adjustGoalPosition(target->trajectory.back().position_W, &goal_pos_)) {
    return false;
  }

  // Check maximum sampling density
  if (p_max_density_range_ > 0.0) {
    double query_pt[3] = {goal_pos_.x(), goal_pos_.y(), goal_pos_.z()};
    std::size_t ret_index[1];
    double out_dist[1];
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(ret_index, out_dist);
    kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));
    if (resultSet.size() > 0) {
      if (out_dist[0] <= p_max_density_range_ * p_max_density_range_) {
        return false;
      }
    }
  }

  // Compute the gain of the new point (evaluation must be single point!)
  TrajectorySegment* new_segment;
  EigenTrajectoryPoint goal_point;
  goal_point.position_W = goal_pos_;
  goal_point.setFromYaw(static_cast<double>(rand()) / RAND_MAX * 2.0 *
                        M_PI);  // random orientation
  if (p_crop_segments_) {
    new_segment = target->spawnChild();
    connectPoses(target->trajectory.back(), goal_point,
                 &(new_segment->trajectory),
                 false);  // collision already checked
  } else {
    new_segment = new TrajectorySegment();
    new_segment->trajectory.push_back(goal_point);
    new_segment->parent = nullptr;
  }
  planner_.getTrajectoryEvaluator().computeGain(new_segment);

  // Find nearby parent candidates
  std::vector<TrajectorySegment*> candidate_parents;
  if (findNearbyCandidates(goal_pos_, &candidate_parents)) {
    rewireToBestParent(new_segment, candidate_parents);
  }

  // Rewire existing segments within range to the new segment, if this increases
  // their value
  if (p_rewire_intermediate_) {
    TrajectorySegment* current = new_segment;
    while (current) {
      // the connection of the new segment to the root cannot be rewired
      // (loops!)
      candidate_parents.erase(std::remove(candidate_parents.begin(),
                                          candidate_parents.end(), current),
                              candidate_parents.end());
      current = current->parent;
    }
    std::vector<TrajectorySegment*> new_parent = {new_segment};
    for (int i = 0; i < candidate_parents.size(); ++i) {
      rewireToBestParent(candidate_parents[i], new_parent);
    }
  }

  // Add to the kdtree
  if (new_segment->parent) {
    tree_data_.addSegment(new_segment);
    kdtree_->addPoints(tree_data_.points.size() - 1,
                       tree_data_.points.size() - 1);
    return true;
  } else {
    delete new_segment;
    return false;
  }
}

bool RRTStar::rewireIntermediate(TrajectorySegment* root) {
  // After updating, try rewire all from inside out
  std::vector<TrajectorySegment*> segments;

  // order w.r.t distance
  root->getTree(&segments);
  std::vector<std::pair<double, TrajectorySegment*>> distance_pairs;

  for (int i = 1; i < segments.size(); ++i) {
    distance_pairs.push_back(
        std::make_pair((segments[i]->trajectory.back().position_W -
                        root->trajectory.back().position_W)
                           .norm(),
                       segments[i]));
  }
  std::sort(distance_pairs.begin(), distance_pairs.end());

  // rewire
  std::vector<TrajectorySegment*> candidate_parents;
  std::vector<TrajectorySegment*> safe_parents;
  for (int i = 0; i < distance_pairs.size(); ++i) {
    candidate_parents.clear();
    if (!findNearbyCandidates(
            distance_pairs[i].second->trajectory.back().position_W,
            &candidate_parents)) {
      continue;
    }
    safe_parents.clear();
    for (int j = 0; j < candidate_parents.size(); ++j) {
      // cannot rewire to own children (loops!)
      TrajectorySegment* current = candidate_parents[j];
      while (true) {
        if (current) {
          if (current == distance_pairs[i].second) {
            break;
          }
          current = current->parent;
        } else {
          safe_parents.push_back(candidate_parents[j]);
          break;
        }
      }
    }
    rewireToBestParent(distance_pairs[i].second, safe_parents);
  }
  return true;
}

bool RRTStar::rewireRoot(TrajectorySegment* root, int* next_segment) {
  if (!p_rewire_root_) {
    return true;
  }
  if (!tree_is_reset_) {
    // Force reset (dangling pointers!)
    resetTree(root);
  }
  tree_is_reset_ = false;

  TrajectorySegment* next_root = root->children[*next_segment].get();

  // Try rewiring non-next segments (to keept their branches alive)
  std::vector<TrajectorySegment*> to_rewire;
  root->getChildren(&to_rewire);
  to_rewire.erase(std::remove(to_rewire.begin(), to_rewire.end(), next_root),
                  to_rewire.end());
  bool rewired_something = true;
  while (rewired_something) {
    rewired_something = false;
    for (int i = 0; i < to_rewire.size(); ++i) {
      if (rewireRootSingle(to_rewire[i], next_root)) {
        to_rewire.erase(to_rewire.begin() + i);
        rewired_something = true;
        break;
      }
    }
  }

  // If necessary (some segments would die) reinsert old root
  if (p_reinsert_root_ && to_rewire.size() > 0) {
    EigenTrajectoryPointVector new_trajectory;
    if ((next_root->trajectory.back().position_W -
         root->trajectory.back().position_W)
            .norm() > 0.0) {
      // don't reinsert zero movement nodes
      if (connectPoses(next_root->trajectory.back(), root->trajectory.back(),
                       &new_trajectory, false)) {
        TrajectorySegment* reinserted_root = next_root->spawnChild();
        reinserted_root->trajectory = new_trajectory;
        // take info from old root (without value since already seen) will be
        // discarded/updated anyways
        reinserted_root->info = std::move(root->info);
        planner_.getTrajectoryEvaluator().computeCost(reinserted_root);
        planner_.getTrajectoryEvaluator().computeValue(reinserted_root);
        tree_data_.addSegment(reinserted_root);
        kdtree_->addPoints(tree_data_.points.size() - 1,
                           tree_data_.points.size() - 1);

        // rewire
        for (TrajectorySegment* segment : to_rewire) {
          for (int i = 0; i < segment->parent->children.size(); ++i) {
            if (segment->parent->children[i].get() == segment) {
              // Move from existing parent
              reinserted_root->children.push_back(
                  std::move(segment->parent->children[i]));
              segment->parent->children.erase(
                  segment->parent->children.begin() + i);
              segment->parent = reinserted_root;
            }
          }
        }
      }
    }
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

bool RRTStar::rewireRootSingle(TrajectorySegment* segment,
                               TrajectorySegment* new_root) {
  // Try rewiring a single segment
  std::vector<TrajectorySegment*> candidate_parents;
  if (!findNearbyCandidates(segment->trajectory.back().position_W,
                            &candidate_parents)) {
    return false;
  }

  // remove all candidates that are still connected to the root
  std::vector<TrajectorySegment*> safe_candidates;
  TrajectorySegment* current;
  for (int i = 0; i < candidate_parents.size(); ++i) {
    current = candidate_parents[i];
    while (current) {
      if (current == new_root) {
        safe_candidates.push_back(candidate_parents[i]);
        break;
      }
      current = current->parent;
    }
  }
  if (safe_candidates.empty()) {
    return false;
  }
  // force rewire
  return rewireToBestParent(segment, safe_candidates, true);
}

bool RRTStar::findNearbyCandidates(const Eigen::Vector3d& target_point,
                                   std::vector<TrajectorySegment*>* result) {
  // Also tried radius search here but that stuff blows for some reason...
  double query_pt[3] = {target_point.x(), target_point.y(), target_point.z()};
  std::size_t ret_index[p_n_neighbors_];  // NOLINT
  double out_dist[p_n_neighbors_];        // NOLINT
  nanoflann::KNNResultSet<double> resultSet(p_n_neighbors_);
  resultSet.init(ret_index, out_dist);
  kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));
  bool candidate_found = false;
  for (int i = 0; i < resultSet.size(); ++i) {
    if (out_dist[i] <= p_max_rewire_range_ * p_max_rewire_range_) {
      candidate_found = true;
      result->push_back(tree_data_.data[ret_index[i]]);
    }
  }
  return candidate_found;
}

bool RRTStar::rewireToBestParent(
    TrajectorySegment* segment,
    const std::vector<TrajectorySegment*>& candidates, bool force_rewire) {
  // Evaluate all candidate parents and store the best one in the segment
  // Goal is the end point of the segment
  EigenTrajectoryPoint goal_point = segment->trajectory.back();

  // store the initial segment
  TrajectorySegment best_segment = segment->shallowCopy();
  TrajectorySegment* initial_parent = segment->parent;

  // Find best segment
  for (TrajectorySegment* candidate : candidates) {
    segment->trajectory.clear();
    segment->parent = candidate;
    if (connectPoses(candidate->trajectory.back(), goal_point,
                     &(segment->trajectory))) {
      // Feasible connection: evaluate the trajectory
      planner_.getTrajectoryEvaluator().computeCost(segment);
      planner_.getTrajectoryEvaluator().computeValue(segment);
      if (best_segment.parent == nullptr || force_rewire ||
          segment->value > best_segment.value) {
        best_segment = segment->shallowCopy();
        force_rewire = false;
      }
    }
  }
  if (best_segment.parent == nullptr) {
    // No connection found and no previous trajectory
    return false;
  } else {
    // Apply best segment and rewire
    segment->parent = best_segment.parent;
    segment->trajectory = best_segment.trajectory;
    segment->cost = best_segment.cost;
    segment->value = best_segment.value;
    planner_.getTrajectoryEvaluator().computeValue(segment);
    if (segment->parent == initial_parent) {
      // Back to old parent
      return ~force_rewire;
    } else if (initial_parent == nullptr) {
      // Found new parent
      segment->parent->children.push_back(
          std::unique_ptr<TrajectorySegment>(segment));
      return true;
    } else {
      for (int i = 0; i < initial_parent->children.size(); ++i) {
        if (initial_parent->children[i].get() == segment) {
          // Move from existing parent
          segment->parent->children.push_back(
              std::move(initial_parent->children[i]));
          initial_parent->children.erase(initial_parent->children.begin() + i);
          // update subtree
          if (p_update_subsequent_) {
            std::vector<TrajectorySegment*> subtree;
            segment->getTree(&subtree);
            for (int j = 1; j < subtree.size(); ++j) {
              planner_.getTrajectoryEvaluator().computeValue(subtree[j]);
            }
          }
          return true;
        }
      }
      // Rewiring failed (should not happen by construction)
      return false;
    }
  }
}

}  // namespace trajectory_generator

namespace trajectory_evaluator {

// RRTStarEvaluatorAdapter (just delegate everything, call rewire on select
// best)
ModuleFactoryRegistry::Registration<RRTStarEvaluatorAdapter>
    RRTStarEvaluatorAdapter::registration("RRTStarEvaluatorAdapter");

RRTStarEvaluatorAdapter::RRTStarEvaluatorAdapter(PlannerI& planner)
    : TrajectoryEvaluator(planner) {}

bool RRTStarEvaluatorAdapter::computeGain(TrajectorySegment* traj_in) {
  return following_evaluator_->computeGain(traj_in);
}

bool RRTStarEvaluatorAdapter::computeCost(TrajectorySegment* traj_in) {
  return following_evaluator_->computeCost(traj_in);
}

bool RRTStarEvaluatorAdapter::computeValue(TrajectorySegment* traj_in) {
  return following_evaluator_->computeValue(traj_in);
}

int RRTStarEvaluatorAdapter::selectNextBest(TrajectorySegment* traj_in) {
  int next = following_evaluator_->selectNextBest(traj_in);
  generator_->rewireRoot(traj_in, &next);
  return next;
}

bool RRTStarEvaluatorAdapter::updateSegment(TrajectorySegment* segment) {
  return following_evaluator_->updateSegment(segment);
}

void RRTStarEvaluatorAdapter::visualizeTrajectoryValue(
    VisualizationMarkers* markers, const TrajectorySegment& trajectory) {
  following_evaluator_->visualizeTrajectoryValue(markers, trajectory);
}

void RRTStarEvaluatorAdapter::setupFromParamMap(Module::ParamMap* param_map) {
  generator_ = dynamic_cast<trajectory_generator::RRTStar*>(
      planner_.getFactory().readLinkableModule("RRTStarGenerator"));
  // Create following evaluator
  std::string args;  // default args extends the parent namespace
  std::string param_ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "following_evaluator_args", &args,
                        param_ns + "/following_evaluator");
  following_evaluator_ =
      planner_.getFactory().createModule<TrajectoryEvaluator>(args, planner_,
                                                              verbose_modules_);

  // setup parent
  TrajectoryEvaluator::setupFromParamMap(param_map);
}

}  // namespace trajectory_evaluator
}  // namespace active_3d_planning
