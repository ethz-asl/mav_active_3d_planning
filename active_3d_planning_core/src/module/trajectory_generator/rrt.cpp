#define _USE_MATH_DEFINES

#include "active_3d_planning_core/module/trajectory_generator/rrt.h"

#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "active_3d_planning_core/data/system_constraints.h"
#include "active_3d_planning_core/data/trajectory.h"

namespace active_3d_planning {
namespace trajectory_generator {

ModuleFactoryRegistry::Registration<RRT> RRT::registration("RRT");

RRT::RRT(PlannerI& planner) : TrajectoryGenerator(planner) {}

void RRT::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<bool>(param_map, "crop_segments", &p_crop_segments_, false);
  setParam<double>(param_map, "crop_margin", &p_crop_margin_, 0.1);
  setParam<double>(param_map, "crop_min_length", &p_crop_min_length_, 0.2);
  setParam<double>(param_map, "sampling_rate", &p_sampling_rate_, 20.0);
  setParam<double>(param_map, "max_extension_range", &p_max_extension_range_,
                   1.0);
  setParam<std::string>(param_map, "sampling_mode", &p_sampling_mode_,
                        std::string("uniform"));
  setParam<bool>(param_map, "sample_yaw", &p_sample_yaw_, true);
  setParam<int>(param_map, "maximum_tries", &p_maximum_tries_, 1000);
  setParam<int>(param_map, "semilocal_sampling_count", &p_semilocal_count_, 10);
  setParam<double>(param_map, "semilocal_sampling_radius_max",
                   &p_semilocal_radius_max_, 1.0);
  setParam<double>(param_map, "semilocal_sampling_radius_min",
                   &p_semilocal_radius_min_, 0.2);
  setParam<double>(param_map, "min_path_length", &p_min_path_length_, 0.0);
  previous_root_ = nullptr;

  // setup parent
  TrajectoryGenerator::setupFromParamMap(param_map);
}

bool RRT::checkParamsValid(std::string* error_message) {
  if (p_sampling_rate_ <= 0.0) {
    *error_message = "sampling_rate expected > 0";
    return false;
  }
  if (p_crop_segments_ && p_crop_margin_ <= 0.0) {
    *error_message = "crop_margin expected > 0";
    return false;
  }
  if (p_sampling_mode_ != "uniform" && p_sampling_mode_ != "spheric" &&
      p_sampling_mode_ != "semilocal") {
    *error_message = "unknown sampling mode '" + p_sampling_mode_ + "'";
    return false;
  }
  if (p_sampling_mode_ == "semilocal" && p_semilocal_count_ < 1) {
    *error_message = "semilocal_sampling_count expected > 0";
    return false;
  }
  if (p_sampling_mode_ == "semilocal" && p_semilocal_radius_max_ <= 0.0) {
    *error_message = "semilocal_sampling_radius_max expected > 0";
    return false;
  }
  if (p_sampling_mode_ == "semilocal" &&
      p_semilocal_radius_max_ < p_semilocal_radius_min_) {
    *error_message =
        "semilocal_sampling_radius_max expected >= "
        "semilocal_sampling_radius_min";
    return false;
  }
  return TrajectoryGenerator::checkParamsValid(error_message);
}

bool RRT::selectSegment(TrajectorySegment** result, TrajectorySegment* root) {
  // If the root has changed, reset the kdtree and populate with the current
  // trajectory tree
  if (previous_root_ != root) {
    resetTree(root);
    previous_root_ = root;
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

  // sample candidate points
  bool goal_found = false;
  Eigen::Vector3d goal_pos;
  int counter = 0;
  while (!goal_found && counter <= p_maximum_tries_) {
    if (p_maximum_tries_ > 0) {
      counter++;
    }
    goal_pos = root->trajectory.back().position_W;
    sampleGoal(&goal_pos);
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
  if (!kdtree_->findNeighbors(resultSet, query_pt,
                              nanoflann::SearchParams(10))) {
    *result = nullptr;
    return false;
  }

  // Valid target found
  goal_pos_ = goal_pos;
  *result = tree_data_.data[ret_index];
  return true;
}

bool RRT::expandSegment(TrajectorySegment* target,
                        std::vector<TrajectorySegment*>* new_segments) {
  if (!target) {
    // Segment selection failed
    return false;
  }

  // Check max segment range and cropping
  if (!adjustGoalPosition(target->trajectory.back().position_W, &goal_pos_)) {
    return false;
  }

  // try creating a trajectory
  EigenTrajectoryPointVector trajectory;
  EigenTrajectoryPoint start_point = target->trajectory.back();
  EigenTrajectoryPoint goal_point;
  goal_point.position_W = goal_pos_;
  if (p_sample_yaw_) {
    goal_point.setFromYaw(static_cast<double>(rand()) / RAND_MAX * 2.0 *
                          M_PI);  // random orientation
  } else {
    goal_point.setFromYaw(
        std::atan2(goal_pos_.x() - start_point.position_W.x(),
                   -goal_pos_.y() + start_point.position_W.y()) -
        M_PI / 2.0);  // face direction of travel
  }
  if (!connectPoses(start_point, goal_point, &trajectory)) {
    return false;
  }

  // Build result and add it to the kdtree
  TrajectorySegment* new_segment = target->spawnChild();
  new_segment->trajectory = trajectory;
  new_segments->push_back(new_segment);
  tree_data_.addSegment(new_segment);
  kdtree_->addPoints(tree_data_.points.size() - 1,
                     tree_data_.points.size() - 1);
  return true;
}

bool RRT::sampleGoal(Eigen::Vector3d* goal_pos) {
  if (p_sampling_mode_ == "uniform") {
    // sample from bounding volume (assumes box atm)
    (*goal_pos)[0] = bounding_volume_->x_min +
                     static_cast<double>(rand()) / RAND_MAX *
                         (bounding_volume_->x_max - bounding_volume_->x_min);
    (*goal_pos)[1] = bounding_volume_->y_min +
                     static_cast<double>(rand()) / RAND_MAX *
                         (bounding_volume_->y_max - bounding_volume_->y_min);
    (*goal_pos)[2] = bounding_volume_->z_min +
                     static_cast<double>(rand()) / RAND_MAX *
                         (bounding_volume_->z_max - bounding_volume_->z_min);
    return true;

  } else if (p_sampling_mode_ == "spheric") {
    // Bircher way (for unbiased sampling, also assumes box atm)
    double radius = std::sqrt(
        std::pow(bounding_volume_->x_max - bounding_volume_->x_min, 2.0) +
        std::pow(bounding_volume_->y_max - bounding_volume_->y_min, 2.0) +
        std::pow(bounding_volume_->z_max - bounding_volume_->z_min, 2.0));
    for (int i = 0; i < 3; i++) {
      (*goal_pos)[i] +=
          2.0 * radius * (static_cast<double>(rand()) / RAND_MAX - 0.5);
    }
    return true;
  } else if (p_sampling_mode_ == "semilocal") {
    if (semilocal_count_ > 0) {
      // Not enough local points found, sample from local sphere
      double theta = 2.0 * M_PI * static_cast<double>(rand()) / RAND_MAX;
      double phi = acos(1.0 - 2.0 * static_cast<double>(rand()) / RAND_MAX);
      double rho = p_semilocal_radius_min_ +
                   (static_cast<double>(rand()) / RAND_MAX) *
                       (p_semilocal_radius_max_ - p_semilocal_radius_min_);
      *goal_pos += rho * Eigen::Vector3d(sin(phi) * cos(theta),
                                         sin(phi) * sin(theta), cos(phi));
      semilocal_count_ -= 1;
      return true;
    } else {
      // sample spheric
      double radius = std::sqrt(
          std::pow(bounding_volume_->x_max - bounding_volume_->x_min, 2.0) +
          std::pow(bounding_volume_->y_max - bounding_volume_->y_min, 2.0) +
          std::pow(bounding_volume_->z_max - bounding_volume_->z_min, 2.0));
      for (int i = 0; i < 3; i++) {
        (*goal_pos)[i] +=
            2.0 * radius * (static_cast<double>(rand()) / RAND_MAX - 0.5);
      }
      return true;
    }
  }
  return false;
}

bool RRT::connectPoses(const EigenTrajectoryPoint& start,
                       const EigenTrajectoryPoint& goal,
                       EigenTrajectoryPointVector* result,
                       bool check_collision) {
  // try creating a linear trajectory
  Eigen::Vector3d start_pos = start.position_W;
  Eigen::Vector3d direction = goal.position_W - start_pos;

  // check minimum length
  if ((start_pos - goal.position_W).norm() < p_min_path_length_) {
    return false;
  }

  // check collision
  int n_points =
      std::ceil(direction.norm() / planner_.getSystemConstraints().v_max *
                    p_sampling_rate_ +
                1);
  if (check_collision) {
    for (int i = 0; i <= n_points; ++i) {
      if (!checkTraversable(start_pos + static_cast<double>(i) /
                                            static_cast<double>(n_points) *
                                            direction)) {
        return false;
      }
    }
  }
  // Build trajectory
  for (int i = 0; i <= n_points; ++i) {
    EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W =
        start_pos +
        static_cast<double>(i) / static_cast<double>(n_points) * direction;
    trajectory_point.setFromYaw(goal.getYaw());
    trajectory_point.time_from_start_ns =
        static_cast<int64_t>(static_cast<double>(i) / p_sampling_rate_ * 1.0e9);
    result->push_back(trajectory_point);
  }
  return true;
}

bool RRT::adjustGoalPosition(const Eigen::Vector3d& start_pos,
                             Eigen::Vector3d* goal_pos_) {
  Eigen::Vector3d direction = *goal_pos_ - start_pos;
  // check min length
  if (direction.norm() < p_min_path_length_) {
    return false;
  }
  if (p_max_extension_range_ > 0.0 &&
      direction.norm() > p_max_extension_range_) {
    // check max length
    direction *= p_max_extension_range_ / direction.norm();
  }
  if (p_crop_segments_) {
    // if the full length cannot be reached, crop it
    int n_points = std::ceil(direction.norm() * p_sampling_rate_ /
                                 planner_.getSystemConstraints().v_max +
                             1);
    for (int i = 0; i <= n_points; ++i) {
      if (!checkTraversable(start_pos + static_cast<double>(i) /
                                            static_cast<double>(n_points) *
                                            direction)) {
        double length = direction.norm() * static_cast<double>(i) /
                            static_cast<double>(n_points) -
                        p_crop_margin_;
        if (length <= p_crop_min_length_) {
          return false;
        }
        direction *= length / direction.norm();
        break;
      }
    }
  }
  *goal_pos_ = start_pos + direction;
  return true;
}

bool RRT::resetTree(TrajectorySegment* root) {
  // Remove everything and populate with the semgents from root
  std::vector<TrajectorySegment*> currrent_tree;
  root->getTree(&currrent_tree);
  tree_data_.clear();
  for (int i = 0; i < currrent_tree.size(); ++i) {
    tree_data_.addSegment(currrent_tree[i]);
  }
  kdtree_ = std::unique_ptr<KDTree>(new KDTree(3, tree_data_));
  kdtree_->addPoints(0, tree_data_.points.size() - 1);
  return true;
}

void RRT::TreeData::clear() {
  points.clear();
  data.clear();
}

void RRT::TreeData::addSegment(TrajectorySegment* to_add) {
  points.push_back(to_add->trajectory.back().position_W);
  data.push_back(to_add);
}

}  // namespace trajectory_generator
}  // namespace active_3d_planning
