#include "active_3d_planning_core/module/trajectory_evaluator/yaw_planning_evaluator.h"

#include <limits>
#include <string>
#include <vector>

#include "active_3d_planning_core/module/module_factory.h"
#include "active_3d_planning_core/planner/planner_I.h"

namespace active_3d_planning {
namespace trajectory_evaluator {

// YawPlanningEvaluator
YawPlanningEvaluator::YawPlanningEvaluator(PlannerI& planner)
    : TrajectoryEvaluator(planner) {}

void YawPlanningEvaluator::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<int>(param_map, "n_directions", &p_n_directions_, 4);
  setParam<bool>(param_map, "select_by_value", &p_select_by_value_, false);
  setParam<double>(param_map, "update_range", &p_update_range_,
                   -1.0);  // default is no updates
  setParam<double>(param_map, "update_gain", &p_update_gain_, 0.0);

  // Register link for yaw planning udpaters
  planner_.getFactory().registerLinkableModule("YawPlanningEvaluator", this);

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

bool YawPlanningEvaluator::checkParamsValid(std::string* error_message) {
  if (p_n_directions_ < 1) {
    *error_message = "n_directions expected > 0";
    return false;
  }
  return TrajectoryEvaluator::checkParamsValid(error_message);
}

bool YawPlanningEvaluator::computeGain(TrajectorySegment* traj_in) {
  // Init
  double start_yaw = traj_in->trajectory.front().getYaw();
  double original_yaw = traj_in->trajectory.back().getYaw();
  traj_in->info = std::make_unique<YawPlanningInfo>();
  YawPlanningInfo* info = static_cast<YawPlanningInfo*>(traj_in->info.get());
  info->active_orientation = 0;

  // Sample all directions
  for (int i = 0; i < p_n_directions_; ++i) {
    info->orientations.push_back(traj_in->shallowCopy());
    setTrajectoryYaw(&(info->orientations.back()), start_yaw,
                     sampleYaw(original_yaw, i));
    following_evaluator_->computeGain(&(info->orientations.back()));
    if (p_select_by_value_) {
      following_evaluator_->computeCost(&(info->orientations.back()));
      following_evaluator_->computeValue(&(info->orientations.back()));
    }
  }

  // Apply best segment to the original one, store rest in info
  setBestYaw(traj_in);
  return true;
}

void YawPlanningEvaluator::setBestYaw(TrajectorySegment* segment) {
  // Select the best yaw out of all current orientations.
  YawPlanningInfo* info = static_cast<YawPlanningInfo*>(segment->info.get());
  double min_gain = std::numeric_limits<double>::max();
  double max_gain = std::numeric_limits<double>::lowest();
  int max_index = 0;
  for (int i = 0; i < info->orientations.size(); ++i) {
    double gain = p_select_by_value_ ? gain = info->orientations[i].value
                                     : gain = info->orientations[i].gain;
    if (gain > max_gain) {
      max_gain = gain;
      max_index = i;
    } else if (gain < min_gain) {
      min_gain = gain;
    }
  }
  double best_yaw;
  if (max_gain > min_gain) {
    // got a best yaw
    best_yaw = defaults::angleScaled(
        info->orientations[max_index].trajectory.back().getYaw());
  } else {
    // indifferent, use direction of travel
    Eigen::Vector3d direction = segment->trajectory.back().position_W -
                                segment->trajectory.front().position_W;
    best_yaw = defaults::angleScaled(std::atan2(direction[1], direction[0]));
  }
  setTrajectoryYaw(segment, segment->trajectory.front().getYaw(), best_yaw);
  info->active_orientation = max_index;

  // Set the gain, and optionally cost and value.
  segment->gain = info->orientations[max_index].gain;
  if (p_select_by_value_) {
    segment->cost = info->orientations[max_index].cost;
    segment->value = info->orientations[max_index].value;
  }
}

bool YawPlanningEvaluator::computeCost(TrajectorySegment* traj_in) {
  return following_evaluator_->computeCost(traj_in);
}

bool YawPlanningEvaluator::computeValue(TrajectorySegment* traj_in) {
  return following_evaluator_->computeValue(traj_in);
}

int YawPlanningEvaluator::selectNextBest(TrajectorySegment* traj_in) {
  return following_evaluator_->selectNextBest(traj_in);
}

bool YawPlanningEvaluator::updateSegment(TrajectorySegment* segment) {
  // Double check whether the trajectory has changed.
  YawPlanningInfo* info =
      reinterpret_cast<YawPlanningInfo*>(segment->info.get());
  if (info) {
    for (size_t i = 0; i < info->orientations.size(); ++i) {
      if (info->orientations[i].parent != segment->parent ||
          info->orientations[i].trajectory.front().position_W !=
              segment->trajectory.front().position_W) {
        double original_yaw = info->orientations[i].trajectory.back().getYaw();
        info->orientations[i].parent = segment->parent;
        info->orientations[i].trajectory = segment->trajectory;
        setTrajectoryYaw(&(info->orientations[i]),
                         segment->trajectory.front().getYaw(), original_yaw);
      }
    }
  }

  // Update the gains if required.
  if (segment->parent && info) {
    double dist =
        (planner_.getCurrentPosition() - segment->trajectory.back().position_W)
            .norm();
    if (p_update_range_ == 0.0 || p_update_range_ > dist) {
      YawPlanningInfo* info =
          reinterpret_cast<YawPlanningInfo*>(segment->info.get());
      for (int i = 0; i < info->orientations.size(); ++i) {
        if (info->orientations[i].gain > p_update_gain_) {
          // all conditions met: update gain of segment
          following_evaluator_->computeGain(&(info->orientations[i]));
        }
      }
    }
    // Update trajectory
    setBestYaw(segment);
  }
  return following_evaluator_->updateSegment(segment);
}

void YawPlanningEvaluator::visualizeTrajectoryValue(
    VisualizationMarkers* markers, const TrajectorySegment& trajectory) {
  if (!trajectory.info) {
    return;
  }
  YawPlanningInfo* info =
      reinterpret_cast<YawPlanningInfo*>(trajectory.info.get());
  // Let the followup evaluator draw the visualization of the selected
  // orientation
  following_evaluator_->visualizeTrajectoryValue(
      markers, info->orientations[info->active_orientation]);
}

}  // namespace trajectory_evaluator
}  // namespace active_3d_planning
