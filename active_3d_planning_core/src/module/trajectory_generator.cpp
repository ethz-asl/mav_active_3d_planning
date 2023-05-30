#include "active_3d_planning_core/module/trajectory_generator.h"

#include <string>

#include "active_3d_planning_core/map/map.h"
#include "active_3d_planning_core/module/module_factory.h"

namespace active_3d_planning {

TrajectoryGenerator::TrajectoryGenerator(PlannerI& planner) : Module(planner) {}

void TrajectoryGenerator::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<bool>(param_map, "collision_optimistic", &p_collision_optimistic_,
                 false);
  setParam<double>(param_map, "clearing_radius", &p_clearing_radius_, 0.0);
  setParam<double>(param_map, "robot_radius", &p_robot_radius_, 1.0); // Avoid sample within robot pose radius
  setParam<std::string>(param_map, "robot_frame_id", &p_robot_frame_id_, "");
  std::string ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "segment_selector_args", &p_selector_args_,
                        ns + "/segment_selector");
  setParam<std::string>(param_map, "generator_updater_args", &p_updater_args_,
                        ns + "/generator_updater");
  std::string temp_args;
  setParam<std::string>(param_map, "bounding_volume_args", &temp_args,
                        ns + "/bounding_volume");
  bounding_volume_ = planner_.getFactory().createModule<BoundingVolume>(
      temp_args, planner_, verbose_modules_);
  setParam<std::string>(param_map, "system_constraints_args", &temp_args,
                        ns + "/system_constraints");
  setParam<size_t>(param_map, "keep_last_n", &p_keep_last_n_, 5);
}

bool TrajectoryGenerator::checkTraversable(const Eigen::Vector3d& position) {
  // check bounding volume
  if (!bounding_volume_->contains(position)) {
    return false;
  }
  //
  if (planner_.getMap().isObserved(position)) {
    return planner_.getMap().isTraversable(position);
  }
  if (p_clearing_radius_ > 0.0) {
    if ((planner_.getCurrentPosition() - position).norm() <
        p_clearing_radius_) {
      return true;
    }
  }
  return p_collision_optimistic_;
}

bool TrajectoryGenerator::checkMultiRobotCollision(const Eigen::Vector3d& position) {
  // Add a mutex for recent_goal_poses_
  std::lock_guard<std::mutex> lock(recent_goal_poses_mutex_);
  // Go through all the recent_goal_poses_ and check if position is within radius
  for (auto it = recent_goal_poses_->begin(); it != recent_goal_poses_->end(); ++it) {

    // Go through all elements in FixedQueue
    for (auto it_q = (it->second).begin(); it_q != (it->second).end(); ++it_q) {
      // Get the pose and quat from element
      Eigen::Vector3d pose = it_q->first;
      Eigen::Vector4d quat = it_q->second;

      // Check if position is within radius and if quat is within yaw margin return true
      if ((pose - position).norm() < p_robot_radius_) {
        return true;
      }
    }

    // // Check if position is within radius, if it is, return true
    // if ((it->second - position).norm() < p_robot_radius_) {
    //   return true;
    // }
  }
  return false;
}

bool TrajectoryGenerator::updateGoals(const std::string& unique_id,
                                      const Eigen::Vector3d& pose,
                                      const Eigen::Vector4d& quat) {
  // Add a mutex for recent_goal_poses_
  std::lock_guard<std::mutex> lock(recent_goal_poses_mutex_);
  if (!recent_goal_poses_) {
    recent_goal_poses_.reset(new std::map<std::string, FixedQueue<EigenVector3d4d>>());
  }

  if (recent_goal_poses_->find(unique_id) == recent_goal_poses_->end()) {
      // The key does not exist in the map, so initialize it.
      (*recent_goal_poses_)[unique_id] = FixedQueue<EigenVector3d4d>(p_keep_last_n_);
  }

  // New EigenVector3d4d as a pair
  EigenVector3d4d pose_quat;
  pose_quat.first = pose;
  pose_quat.second = quat;

  // Push the pose into the FixedQueue
  (*recent_goal_poses_)[unique_id].push(pose_quat);
  return true;
}

bool TrajectoryGenerator::selectSegment(TrajectorySegment** result,
                                        TrajectorySegment* root) {
  // If not implemented use a (default) module
  if (!segment_selector_) {
    segment_selector_ = planner_.getFactory().createModule<SegmentSelector>(
        p_selector_args_, planner_, verbose_modules_);
  }
  return segment_selector_->selectSegment(result, root);
}

bool TrajectoryGenerator::updateSegment(TrajectorySegment* segment) {
  // If not implemented use a (default) module
  if (!generator_updater_) {
    generator_updater_ = planner_.getFactory().createModule<GeneratorUpdater>(
        p_updater_args_, planner_, verbose_modules_);
  }
  return generator_updater_->updateSegment(segment);
}

bool TrajectoryGenerator::extractTrajectoryToPublish(
    EigenTrajectoryPointVector* trajectory, const TrajectorySegment& segment) {
  // Default does not manipulate the stored trajectory
  *trajectory = segment.trajectory;
  return true;
}

SegmentSelector::SegmentSelector(PlannerI& planner) : Module(planner) {}

GeneratorUpdater::GeneratorUpdater(PlannerI& planner) : Module(planner) {}
}  // namespace active_3d_planning
