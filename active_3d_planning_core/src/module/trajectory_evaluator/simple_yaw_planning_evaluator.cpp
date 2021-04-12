#define _USE_MATH_DEFINES

#include "active_3d_planning_core/module/trajectory_evaluator/simple_yaw_planning_evaluator.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "active_3d_planning_core/tools/defaults.h"

namespace active_3d_planning {
namespace trajectory_evaluator {

ModuleFactoryRegistry::Registration<SimpleYawPlanningEvaluator>
    SimpleYawPlanningEvaluator::registration("SimpleYawPlanningEvaluator");

SimpleYawPlanningEvaluator::SimpleYawPlanningEvaluator(PlannerI& planner)
    : YawPlanningEvaluator(planner) {}

double SimpleYawPlanningEvaluator::sampleYaw(double original_yaw,
                                             int sample_number) {
  // Uniform sampling
  return defaults::angleScaled(original_yaw +
                               static_cast<double>(sample_number) * 2.0 * M_PI /
                                   p_n_directions_);
}

void SimpleYawPlanningEvaluator::setTrajectoryYaw(TrajectorySegment* segment,
                                                  double start_yaw,
                                                  double target_yaw) {
  // just set the yaw of the entire trajectory to the sampled value
  for (int i = 0; i < segment->trajectory.size(); ++i) {
    segment->trajectory[i].setFromYaw(target_yaw);
  }
}

void SimpleYawPlanningEvaluator::setupFromParamMap(
    Module::ParamMap* param_map) {
  setParam<bool>(param_map, "visualize_followup", &p_visualize_followup_, true);
  // setup parent
  YawPlanningEvaluator::setupFromParamMap(param_map);
}

void SimpleYawPlanningEvaluator::visualizeTrajectoryValue(
    VisualizationMarkers* markers, const TrajectorySegment& trajectory) {
  if (!trajectory.info) {
    return;
  }
  YawPlanningInfo* info =
      reinterpret_cast<YawPlanningInfo*>(trajectory.info.get());
  // Simple version: visualize facing of endpoints of trajectories, colored with
  // value (highest green to lowest red)
  double max_value = info->orientations[0].gain;
  double min_value = info->orientations[0].gain;
  if (p_select_by_value_) {
    max_value = info->orientations[0].value;
    min_value = info->orientations[0].value;
  }
  for (int i = 1; i < info->orientations.size(); ++i) {
    if (p_select_by_value_) {
      if (info->orientations[i].value > max_value) {
        max_value = info->orientations[i].value;
      }
      if (info->orientations[i].value < min_value) {
        min_value = info->orientations[i].value;
      }
    } else {
      if (info->orientations[i].gain > max_value) {
        max_value = info->orientations[i].gain;
      }
      if (info->orientations[i].gain < min_value) {
        min_value = info->orientations[i].gain;
      }
    }
  }
  for (int i = 0; i < info->orientations.size(); ++i) {
    // Setup marker message
    VisualizationMarker marker;
    marker.position = info->orientations[i].trajectory.back().position_W;
    marker.orientation =
        info->orientations[i].trajectory.back().orientation_W_B;
    marker.type = VisualizationMarker::ARROW;
    marker.scale.x() = 0.6;
    marker.scale.y() = 0.07;
    marker.scale.z() = 0.07;
    marker.action = VisualizationMarker::ADD;

    // Color according to relative value (blue when indifferent)
    if (max_value != min_value) {
      double frac =
          (info->orientations[i].gain - min_value) / (max_value - min_value);
      if (p_select_by_value_) {
        frac =
            (info->orientations[i].value - min_value) / (max_value - min_value);
      }
      marker.color.r = std::min((0.5 - frac) * 2.0 + 1.0, 1.0);
      marker.color.g = std::min((frac - 0.5) * 2.0 + 1.0, 1.0);
      marker.color.b = 0.0;
    } else {
      marker.color.r = 0.3;
      marker.color.g = 0.3;
      marker.color.b = 1.0;
    }
    marker.color.a = 0.4;
    markers->addMarker(marker);
  }

  // Followup
  if (p_visualize_followup_) {
    following_evaluator_->visualizeTrajectoryValue(
        markers, info->orientations[info->active_orientation]);
  }
}

}  // namespace trajectory_evaluator
}  // namespace active_3d_planning
