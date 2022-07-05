#define _USE_MATH_DEFINES

#include "active_3d_planning_core/planner/online_planner.h"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace active_3d_planning {

OnlinePlanner::OnlinePlanner(ModuleFactory* factory,
                             Module::ParamMap* param_map)
    : factory_(factory), running_(false), planning_(false) {
  // Setup params
  OnlinePlanner::setupFromParamMap(param_map);
}

OnlinePlanner::~OnlinePlanner() {
  // shutdown filestream
  if (perf_log_file_.is_open()) {
    perf_log_file_.close();
  }
}

void OnlinePlanner::setupFromParamMap(Module::ParamMap* param_map) {
  // Setup Params
  bool verbose_modules;
  bool build_modules_on_init;

  // Logging, printing and visualization
  setParam<bool>(param_map, "verbose", &p_verbose_, true);
  setParam<bool>(param_map, "verbose_modules", &verbose_modules, true);
  setVerbose(verbose_modules);
  setParam<bool>(param_map, "build_modules_on_init", &build_modules_on_init,
                 false);
  setParam<bool>(param_map, "visualize", &p_visualize_, true);
  setParam<bool>(param_map, "log_performance", &p_log_performance_, false);
  setParam<bool>(param_map, "visualize_gain", &p_visualize_gain_, false);
  setParam<bool>(param_map, "highlight_executed_trajectory",
                 &p_highlight_executed_trajectory_, false);

  // Sampling constraints
  setParam<int>(param_map, "max_new_segments", &p_max_new_segments_,
                0);  // set 0 for infinite
  setParam<int>(param_map, "min_new_segments", &p_min_new_segments_, 0);
  setParam<int>(param_map, "max_new_tries", &p_max_new_tries_,
                0);  // set 0 for infinite
  setParam<int>(param_map, "min_new_tries", &p_min_new_tries_, 0);
  setParam<double>(param_map, "min_new_value", &p_min_new_value_, 0.0);
  setParam<int>(param_map, "expand_batch", &p_expand_batch_, 1);
  p_expand_batch_ = std::max(p_expand_batch_, 1);

  // Setup members
  std::string args;  // default args extends the parent namespace
  std::string param_ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "system_constraint_args", &args,
                        param_ns + "/system_constraints");
  system_constraints_ = getFactory().createModule<SystemConstraints>(
      args, *this, verbose_modules);
  setParam<std::string>(param_map, "map_args", &args, param_ns + "/map");
  map_ = getFactory().createModule<Map>(args, *this, verbose_modules);
  setParam<std::string>(param_map, "trajectory_generator_args", &args,
                        param_ns + "/trajectory_generator");
  trajectory_generator_ = getFactory().createModule<TrajectoryGenerator>(
      args, *this, verbose_modules);
  setParam<std::string>(param_map, "trajectory_evaluator_args", &args,
                        param_ns + "/trajectory_evaluator");
  trajectory_evaluator_ = getFactory().createModule<TrajectoryEvaluator>(
      args, *this, verbose_modules);
  setParam<std::string>(param_map, "back_tracker_args", &args,
                        param_ns + "/back_tracker");
  back_tracker_ =
      getFactory().createModule<BackTracker>(args, *this, verbose_modules);

  // Setup performance log
  if (p_log_performance_) {
    std::string log_dir("");
    setParam<std::string>(param_map, "performance_log_dir", &log_dir, "");
    if (log_dir == "") {
      // Wait for this param until planning start so it can be set after
      // constructor (e.g. from other nodes)
      printWarning(
          "No directory for performance log set, performance log turned off.");
      p_log_performance_ = false;
    } else {
      logfile_ = log_dir + "/performance_log.csv";
      perf_log_file_.open(logfile_.c_str());
      perf_log_data_ =
          std::vector<double>(5, 0.0);  // select, expand, gain, cost, value
      perf_cpu_timer_ = std::clock();
      perf_log_file_
          << "RunTime,NTrajectories,NTrajAfterUpdate,Select,Expand,Gain,"
             "Cost,Value,NextBest,UpdateTG,UpdateTE,Visualization,Total";
    }
  }

  // Force lazy initialization of modules (call every function once)
  if (build_modules_on_init) {
    // Empty set of arguments required to run everything
    TrajectorySegment temp_segment;
    TrajectorySegment* temp_pointer;
    std::vector<TrajectorySegment*> temp_vector;
    EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W = Eigen::Vector3d(0, 0, 0);
    trajectory_point.setFromYaw(0);
    temp_segment.trajectory.push_back(trajectory_point);
    temp_segment.spawnChild()->trajectory.push_back(trajectory_point);
    trajectory_generator_->selectSegment(&temp_pointer, &temp_segment);
    trajectory_generator_->expandSegment(temp_pointer, &temp_vector);
    trajectory_generator_->updateSegment(&temp_segment);

    temp_segment = TrajectorySegment();
    temp_segment.trajectory.push_back(trajectory_point);
    trajectory_evaluator_->computeGain(&temp_segment);
    trajectory_evaluator_->computeCost(&temp_segment);
    trajectory_evaluator_->computeValue(&temp_segment);
    trajectory_evaluator_->updateSegment(&temp_segment);

    temp_segment.spawnChild()->trajectory.push_back(trajectory_point);
    trajectory_evaluator_->selectNextBest(&temp_segment);
  }
}

// logging and printing: default to std::cout in worst case
void OnlinePlanner::printInfo(const std::string& text) {
  std::cout << "Info: " << text << std::endl;
}

void OnlinePlanner::printWarning(const std::string& text) {
  std::cout << "Warning: " << text << std::endl;
}

void OnlinePlanner::printError(const std::string& text) {
  std::cout << "Error: " << text << std::endl;
}

void OnlinePlanner::initializePlanning() {
  // Setup initial trajectory Segment at current location
  target_position_ = current_position_;
  target_yaw_ = yawFromQuaternion(current_orientation_);
  current_segment_ =
      std::unique_ptr<TrajectorySegment>(new TrajectorySegment());
  EigenTrajectoryPoint trajectory_point;
  trajectory_point.position_W = target_position_;
  trajectory_point.setFromYaw(target_yaw_);
  current_segment_->trajectory.push_back(trajectory_point);

  // Setup counters
  info_timing_ = std::clock();
  info_count_ = 1;
  info_killed_next_ = 0;
  info_killed_update_ = 0;
  new_segments_ = 0;
  new_segment_tries_ = 0;
  min_new_value_reached_ = p_min_new_value_ == 0.0;
  vis_completed_count_ = 0;
  std::fill(perf_log_data_.begin(), perf_log_data_.begin() + 5, 0.0);

  // Launch expansion as current goal is considered reached
  target_reached_ = true;
}

void OnlinePlanner::planningLoop() {
  // This is the main loop, adjust updating and implement exit conditions
  running_ = true;
  while (running_) {
    if (planning_) {
      loopIteration();
    }
    // [check update /exit conditions here]
  }
}

void OnlinePlanner::loopIteration() {
  // Continuosly expand the trajectory space
  for (int i = 0; i < p_expand_batch_; ++i) {
    expandTrajectories();
  }

  if (!min_new_value_reached_) {
    //  recursively check whether the minimum value is reached
    min_new_value_reached_ = checkMinNewValue(current_segment_);
  }

  // After finishing the current segment, execute the next one
  if (target_reached_) {
    if (new_segment_tries_ >= p_max_new_tries_ && p_max_new_tries_ > 0) {
      // Maximum tries reached: force next segment
      requestNextTrajectory();
    } else {
      if (new_segment_tries_ < p_min_new_tries_) {
        return;  // check minimum tries reached
      }
      if (new_segments_ < p_min_new_segments_) {
        return;  // check minimum successful expansions reached
      }
      if (!min_new_value_reached_) {
        return;  // check minimum value reached
      }
      // All requirements met
      requestNextTrajectory();
    }
  }
}

void OnlinePlanner::verifyTree(TrajectorySegment* next_segment) {
  std::vector<TrajectorySegment*> segments;
  current_segment_->getTree(&segments);
  int counter = 0;
  const int num_segments = segments.size() - 1;
  for (TrajectorySegment* segment : segments) {
    // Check all non-root segments
    if (segment->parent) {
      counter++;
      // Check connectedness.
      constexpr float distance_tolerance = 0.1f;  // 10 cm
      const float distance = (segment->trajectory.front().position_W -
                              segment->parent->trajectory.back().position_W)
                                 .norm();
      if (distance >= distance_tolerance) {
        std::cout << "Segment " << counter << "/" << num_segments
                  << (segment == next_segment ? " (NEXT SEGMENT)" : "")
                  << " is disconnected (d=" << distance << ")." << std::endl;
      }

      // Check collision.
      for (const auto& point : segment->trajectory) {
        if (!trajectory_generator_->checkTraversable(point.position_W)) {
          std::cout << "Segment " << counter << "/" << num_segments
                    << (segment == next_segment ? " (NEXT SEGMENT)" : "")
                    << " is colliding." << std::endl;
          break;
        }
      }
    }
  }
}

bool OnlinePlanner::requestNextTrajectory() {
  if (current_segment_->children.empty()) {
    // No trajectories available: call the backtracker
    back_tracker_->trackBack(current_segment_.get());
    return false;
  }

  // Performance tracking
  double perf_runtime;
  double perf_vis = 0.0;
  double perf_next;
  double perf_uptg;
  double perf_upte;
  std::clock_t timer;
  if (p_log_performance_) {
    timer = std::clock();
  }

  // Visualize candidates
  std::vector<TrajectorySegment*> trajectories_to_vis;
  current_segment_->getTree(&trajectories_to_vis);
  trajectories_to_vis.erase(
      trajectories_to_vis.begin());  // remove current segment (root)
  if (p_visualize_) {
    publishTrajectoryVisualization(trajectories_to_vis);
  }
  int num_trajectories = trajectories_to_vis.size();
  if (p_verbose_ || p_log_performance_) {
    perf_runtime =
        std::chrono::duration<double>(std::clock() - info_timing_).count() /
        1e6;
    info_timing_ = std::clock();
    if (p_verbose_) {
      std::stringstream ss;
      ss << "Replanning!\n(" << std::setprecision(3) << perf_runtime
         << "s elapsed, " << num_trajectories - info_count_ + 1 << " new, "
         << num_trajectories << " total, " << info_killed_next_ + 1
         << " killed by root change, " << info_killed_update_
         << " killed while updating)";
      printInfo(ss.str());
    }
  }
  if (p_log_performance_) {
    perf_vis += static_cast<double>(std::clock() - timer) / CLOCKS_PER_SEC;
    timer = std::clock();
  }

  // Select best next trajectory and update root
  int next_segment =
      trajectory_evaluator_->selectNextBest(current_segment_.get());

  current_segment_ = std::move(current_segment_->children[next_segment]);
  current_segment_->parent = nullptr;
  current_segment_->gain = 0.0;
  current_segment_->cost = 0.0;
  current_segment_->value = 0.0;
  if (p_log_performance_) {
    perf_next = static_cast<double>(std::clock() - timer) / CLOCKS_PER_SEC;
  }
  trajectories_to_vis.clear();
  current_segment_->getTree(&trajectories_to_vis);
  info_killed_next_ = num_trajectories - trajectories_to_vis.size();

  // Move
  EigenTrajectoryPointVector trajectory;
  trajectory_generator_->extractTrajectoryToPublish(&trajectory,
                                                    *current_segment_);
  current_segment_->trajectory = trajectory;

  requestMovement(trajectory);
  target_position_ = trajectory.back().position_W;
  target_yaw_ = trajectory.back().getYaw();
  back_tracker_->segmentIsExecuted(*current_segment_);

  // Visualize
  if (p_log_performance_) {
    timer = std::clock();
  }
  if (p_visualize_) {
    publishEvalVisualization(*current_segment_);
    publishCompletedTrajectoryVisualization(*current_segment_);
  }
  if (p_log_performance_) {
    perf_vis += static_cast<double>(std::clock() - timer) / CLOCKS_PER_SEC;
    timer = std::clock();
  }

  // Update tree
  // recursive tree pass from root to leaves
  updateGeneratorStep(current_segment_.get());
  if (p_log_performance_) {
    perf_uptg = static_cast<double>(std::clock() - timer) / CLOCKS_PER_SEC;
    timer = std::clock();
  }
  updateEvaluatorStep(current_segment_.get());
  if (p_log_performance_) {
    perf_upte = static_cast<double>(std::clock() - timer) / CLOCKS_PER_SEC;
  }
  trajectories_to_vis.clear();
  current_segment_->getTree(&trajectories_to_vis);
  info_killed_update_ =
      num_trajectories - trajectories_to_vis.size() - info_killed_next_;

  // Performance log and printing
  if (p_verbose_ || p_log_performance_) {
    trajectories_to_vis.clear();
    current_segment_->getTree(&trajectories_to_vis);
    info_count_ = trajectories_to_vis.size();

    if (p_log_performance_) {
      perf_log_file_ << "\n"
                     << perf_runtime << "," << num_trajectories << ","
                     << info_count_ << "," << perf_log_data_[0] << ","
                     << perf_log_data_[1] << "," << perf_log_data_[2] << ","
                     << perf_log_data_[3] << "," << perf_log_data_[4] << ","
                     << perf_next << "," << perf_uptg << "," << perf_upte << ","
                     << perf_vis << ","
                     << static_cast<double>(std::clock() - perf_cpu_timer_) /
                            CLOCKS_PER_SEC;
      std::fill(perf_log_data_.begin(), perf_log_data_.begin() + 5,
                0.0);  // reset count
      perf_cpu_timer_ = std::clock();
    }
  }

  // Update tracking values
  new_segment_tries_ = 0;
  new_segments_ = 0;
  min_new_value_reached_ = p_min_new_value_ == 0.0;
  target_reached_ = false;
  return true;
}

void OnlinePlanner::expandTrajectories() {
  // Check max number of tries already and min value found
  if (p_max_new_segments_ > 0 && new_segments_ >= p_max_new_segments_ &&
      min_new_value_reached_) {
    return;
  }

  // Select expansion target
  std::clock_t timer;
  if (p_log_performance_) {
    timer = std::clock();
  }
  TrajectorySegment* expansion_target;
  trajectory_generator_->selectSegment(&expansion_target,
                                       current_segment_.get());
  if (p_log_performance_) {
    perf_log_data_[0] +=
        static_cast<double>(std::clock() - timer) / CLOCKS_PER_SEC;
    timer = std::clock();
  }

  // Expand the target
  std::vector<TrajectorySegment*> created_segments;
  bool success =
      trajectory_generator_->expandSegment(expansion_target, &created_segments);
  if (p_log_performance_) {
    perf_log_data_[1] +=
        static_cast<double>(std::clock() - timer) / CLOCKS_PER_SEC;
    timer = std::clock();
  }
  new_segment_tries_++;
  if (success) {
    new_segments_++;
  }

  // Evaluate newly added segments: Gain
  for (int i = 0; i < created_segments.size(); ++i) {
    trajectory_evaluator_->computeGain(created_segments[i]);
  }
  if (p_log_performance_) {
    perf_log_data_[2] +=
        static_cast<double>(std::clock() - timer) / CLOCKS_PER_SEC;
    timer = std::clock();
  }

  // Costs
  for (int i = 0; i < created_segments.size(); ++i) {
    trajectory_evaluator_->computeCost(created_segments[i]);
  }
  if (p_log_performance_) {
    perf_log_data_[3] +=
        static_cast<double>(std::clock() - timer) / CLOCKS_PER_SEC;
    timer = std::clock();
  }

  // Final value
  for (int i = 0; i < created_segments.size(); ++i) {
    trajectory_evaluator_->computeValue(created_segments[i]);
  }
  if (p_log_performance_) {
    perf_log_data_[4] +=
        static_cast<double>(std::clock() - timer) / CLOCKS_PER_SEC;
  }
}

void OnlinePlanner::publishTrajectoryVisualization(
    const std::vector<TrajectorySegment*>& trajectories) {
  // Display all trajectories in the input and erase previous ones
  double max_value = trajectories[0]->value;
  double min_value = trajectories[0]->value;
  double max_gain = trajectories[0]->gain;
  double min_gain = trajectories[0]->gain;
  TrajectorySegment* goal = nullptr;
  int goal_depth = 0;

  // Find highest gains and values.
  for (int i = 1; i < trajectories.size(); ++i) {
    if (trajectories[i]->value >= max_value) {
      max_value = trajectories[i]->value;
    }
    if (trajectories[i]->value < min_value) {
      min_value = trajectories[i]->value;
    }
    if (trajectories[i]->gain > max_gain) {
      max_gain = trajectories[i]->gain;
    }
    if (trajectories[i]->gain < min_gain) {
      min_gain = trajectories[i]->gain;
    }
  }

  VisualizationMarkers value_markers, gain_markers, text_markers, goal_markers;
  VisualizationMarker msg;
  for (int i = 0; i < trajectories.size(); ++i) {
    // Setup marker message
    msg = VisualizationMarker();
    msg.orientation.w() = 1.0;
    msg.type = VisualizationMarker::POINTS;
    msg.id = i;
    msg.ns = "candidate_trajectories";
    msg.scale.x() = 0.04;
    msg.scale.y() = 0.04;
    msg.color.a = 0.4;
    msg.action = VisualizationMarker::OVERWRITE;

    // Color according to relative value (blue when indifferent)
    if (max_value != min_value) {
      double frac =
          (trajectories[i]->value - min_value) / (max_value - min_value);
      msg.color.r = std::min((0.5 - frac) * 2.0 + 1.0, 1.0);
      msg.color.g = std::min((frac - 0.5) * 2.0 + 1.0, 1.0);
      msg.color.b = 0.0;

      // Identify outmost goal pose.
      if (trajectories[i]->value == max_value) {
        int depth = 0;
        TrajectorySegment* current = trajectories[i];
        while (current) {
          depth++;
          current = current->parent;
        }

        if (depth > goal_depth) {
          goal = trajectories[i];
          goal_depth = depth;
        }
      }
    } else {
      msg.color.r = 0.3;
      msg.color.g = 0.3;
      msg.color.b = 1.0;
    }

    // points
    for (int j = 0; j < trajectories[i]->trajectory.size(); ++j) {
      msg.points.push_back(trajectories[i]->trajectory[j].position_W);
    }
    value_markers.addMarker(msg);

    // visualize gain
    if (p_visualize_gain_) {
      msg = VisualizationMarker();
      msg.orientation.w() = 1.0;
      msg.type = VisualizationMarker::SPHERE;
      msg.action = VisualizationMarker::OVERWRITE;
      msg.id = i;
      msg.ns = "canidate_gains";
      msg.scale.x() = 0.15;
      msg.scale.y() = 0.15;
      msg.scale.z() = 0.15;
      msg.position = trajectories[i]->trajectory.back().position_W;

      // Color according to relative value (blue when indifferent)
      if (min_gain != max_gain) {
        double frac =
            (trajectories[i]->gain - min_gain) / (max_gain - min_gain);
        msg.color.r = std::min((0.5 - frac) * 2.0 + 1.0, 1.0);
        msg.color.g = std::min((frac - 0.5) * 2.0 + 1.0, 1.0);
        msg.color.b = 0.0;
      } else {
        msg.color.r = 0.3;
        msg.color.g = 0.3;
        msg.color.b = 1.0;
      }
      msg.color.a = 1.0;
      gain_markers.addMarker(msg);
    }

    // Text
    msg = VisualizationMarker();
    msg.type = VisualizationMarker::TEXT_VIEW_FACING;
    msg.id = i;
    msg.ns = "candidate_text";
    msg.scale.z() = 0.2;
    msg.color.r = 0.0f;
    msg.color.g = 0.0f;
    msg.color.b = 0.0f;
    msg.color.a = 1.0;
    msg.position = trajectories[i]->trajectory.back().position_W;
    std::stringstream stream;
    stream << std::fixed << std::setprecision(1) << trajectories[i]->gain << "/"
           << std::fixed << std::setprecision(1) << trajectories[i]->cost << "/"
           << std::fixed << std::setprecision(1) << trajectories[i]->value;
    msg.text = stream.str();
    msg.action = VisualizationMarker::OVERWRITE;
    text_markers.addMarker(msg);
  }
  // visualize goal
  msg = VisualizationMarker();
  msg.orientation.w() = 1.0;
  msg.type = VisualizationMarker::POINTS;
  msg.id = 0;
  msg.ns = "current_goal";
  msg.scale.x() = 0.1;
  msg.scale.y() = 0.1;
  msg.color.r = 0.0;
  msg.color.g = 0.9;
  msg.color.b = 0.0;
  msg.color.a = 1.0;
  msg.action = VisualizationMarker::OVERWRITE;

  while (goal) {
    // points
    if (goal->parent) {
      for (const EigenTrajectoryPoint& point : goal->trajectory) {
        msg.points.push_back(point.position_W);
      }
    }
    goal = goal->parent;
  }

  goal_markers.addMarker(msg);

  // visualize
  publishVisualization(value_markers);
  publishVisualization(gain_markers);
  publishVisualization(text_markers);
  publishVisualization(goal_markers);
}

void OnlinePlanner::publishCompletedTrajectoryVisualization(
    const TrajectorySegment& trajectories) {
  // Continuously increment the already traveled path
  VisualizationMarker msg;
  msg.orientation.w() = 1.0;
  msg.type = VisualizationMarker::POINTS;
  msg.ns = "completed_trajectory";
  msg.id = vis_completed_count_;
  msg.action = VisualizationMarker::ADD;
  if (p_highlight_executed_trajectory_) {
    msg.scale.x() = 0.1;
    msg.scale.y() = 0.1;
    msg.color.r = 1.0;
    msg.color.g = 0.0;
    msg.color.b = 0.0;
    msg.color.a = 1.0;
  } else {
    msg.scale.x() = 0.03;
    msg.scale.y() = 0.03;
    msg.color.r = 0.5;
    msg.color.g = 0.5;
    msg.color.b = 0.5;
    msg.color.a = 0.8;
  }

  // points
  for (const EigenTrajectoryPoint& point : trajectories.trajectory) {
    msg.points.push_back(point.position_W);
  }
  VisualizationMarkers array_msg;
  array_msg.addMarker(msg);
  publishVisualization(array_msg);
  vis_completed_count_++;
}

void OnlinePlanner::publishEvalVisualization(
    const TrajectorySegment& trajectory) {
  // Visualize the gain of the current segment
  VisualizationMarkers msg;
  trajectory_evaluator_->visualizeTrajectoryValue(&msg, trajectory);
  int i = 0;
  for (VisualizationMarker& marker : msg.getMarkers()) {
    marker.id = i;
    i++;
    marker.ns = "trajectory_evaluation";
    marker.action = VisualizationMarker::OVERWRITE;
  }
  publishVisualization(msg);
}

bool OnlinePlanner::checkMinNewValue(
    const std::unique_ptr<TrajectorySegment>& segment) {
  // Recursively check whether the minimum value is reached
  if (segment->value >= p_min_new_value_) {
    return true;
  } else {
    for (int i = 0; i < segment->children.size(); ++i) {
      if (checkMinNewValue(segment->children[i])) {
        return true;
      }
    }
  }
  return false;
}

void OnlinePlanner::updateGeneratorStep(TrajectorySegment* target) {
  // Recursive removal
  int j = 0;
  for (int i = 0; i < target->children.size(); ++i) {
    if (trajectory_generator_->updateSegment(target->children[j].get())) {
      j++;
    } else {
      target->children.erase(target->children.begin() + j);
    }
  }
  // remaining children
  for (int i = 0; i < target->children.size(); ++i) {
    updateGeneratorStep(target->children[i].get());
  }
}

void OnlinePlanner::updateEvaluatorStep(TrajectorySegment* target) {
  // Recursive removal
  int j = 0;
  for (int i = 0; i < target->children.size(); ++i) {
    if (trajectory_evaluator_->updateSegment(target->children[j].get())) {
      j++;
    } else {
      target->children.erase(target->children.begin() + j);
    }
  }
  // remaining children
  for (int i = 0; i < target->children.size(); ++i) {
    updateEvaluatorStep(target->children[i].get());
  }
}

}  // namespace active_3d_planning
