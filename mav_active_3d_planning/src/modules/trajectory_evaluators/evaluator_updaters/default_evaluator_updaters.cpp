#include "mav_active_3d_planning/modules/trajectory_evaluators/evaluator_updaters/default_evaluator_updaters.h"

#include <vector>
#include <algorithm>
#include <random>

namespace mav_active_3d_planning {
    namespace evaluator_updaters {

        // ResetTree
        bool ResetTree::updateSegments(TrajectorySegment *root) {
            root->children.clear();
            return true;
        }

        // UpdateAll
        UpdateAll::UpdateAll(bool update_gain, bool update_cost, bool update_value,
                             std::unique_ptr <EvaluatorUpdater> following_updater)
                : update_gain_(update_gain),
                  update_cost_(update_cost),
                  update_value_(update_value) {
            following_updater_ = std::move(following_updater);
        }

        bool UpdateAll::updateSegments(TrajectorySegment *root) {
            // recursively update all segments from root to leaves (as in the planner)
            updateSingle(root);
            return following_updater_->updateSegments(root);
        }

        void UpdateAll::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<bool>(param_map, "update_gain", &update_gain_, true);
            setParam<bool>(param_map, "update_cost", &update_cost_, true);
            setParam<bool>(param_map, "update_value", &update_value_, true);

            // Create Following updater (default does nothing)
            std::string args;   // default args extends the parent namespace
            std::string param_ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "following_updater_args", &args, param_ns + "/following_updater");
            following_updater_ = ModuleFactory::Instance()->createEvaluatorUpdater(args, parent_, verbose_modules_);
        }

        void UpdateAll::updateSingle(TrajectorySegment *segment) {
            if (update_gain_) { parent_->computeGain(segment); }
            if (update_cost_) { parent_->computeCost(segment); }
            if (update_value_) { parent_->computeValue(segment); }
            for (int i = 0; i < segment->children.size(); ++i) {
                updateSingle(segment->children[i].get());
            }
        }

        // PruneByValue
        PruneByValue::PruneByValue(double minimum_value, bool use_relative_values, bool include_subsequent,
                                   std::unique_ptr <EvaluatorUpdater> following_updater)
                : minimum_value_(minimum_value),
                  use_relative_values_(use_relative_values),
                  include_subsequent_(include_subsequent) {
            following_updater_ = std::move(following_updater);
        }

        bool PruneByValue::updateSegments(TrajectorySegment *root) {
            double absolute_minimum = minimum_value_;
            if (use_relative_values_) {
                // Scale to 0 at minimum, 1 at maximum
                std::vector < TrajectorySegment * > segments;
                if (include_subsequent_) {
                    root->getTree(&segments);
                    segments.erase(segments.begin());
                } else {
                    root->getChildren(&segments);
                }
                double min_value = (*std::min_element(segments.begin(), segments.end(),
                                                      TrajectorySegment::comparePtr))->value;
                double max_value = (*std::max_element(segments.begin(), segments.end(),
                                                      TrajectorySegment::comparePtr))->value;
                absolute_minimum = minimum_value_ * (max_value - min_value) + min_value;
            }
            if (include_subsequent_) {
                // remove  recursively (exluding the root itself)
                removeSingle(root, absolute_minimum);
            } else {
                int j = 0;
                for (int i = 0; i < root->children.size(); ++i) {
                    if (root->children[j]->value < absolute_minimum) {
                        root->children.erase(root->children.begin() + j);
                    } else {
                        j++;
                    }
                }
            }
            return following_updater_->updateSegments(root);
        }

        void PruneByValue::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<double>(param_map, "minimum_value", &minimum_value_, 0.0);
            setParam<bool>(param_map, "use_relative_values", &use_relative_values_, false);
            setParam<bool>(param_map, "include_subsequent", &include_subsequent_, true);

            // Create Following updater (default does nothing)
            std::string args;   // default args extends the parent namespace
            std::string param_ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "following_updater_args", &args, param_ns + "/following_updater");
            following_updater_ = ModuleFactory::Instance()->createEvaluatorUpdater(args, parent_, verbose_modules_);
        }

        bool PruneByValue::removeSingle(TrajectorySegment *segment, double min_value) {
            if (segment->children.empty()) {
                return (segment->value < min_value);
            }
            int j = 0;
            for (int i = 0; i < segment->children.size(); ++i) {
                if (removeSingle(segment->children[j].get(), min_value)) {
                    segment->children.erase(segment->children.begin() + j);
                } else {
                    j++;
                }
            }
            return (segment->children.empty() & segment->value < min_value);
        }

        // Periodic
        Periodic::Periodic(double minimum_wait_time, int minimum_wait_calls,
                           std::unique_ptr <EvaluatorUpdater> following_updater)
                : p_minimum_wait_time_(minimum_wait_time),
                  p_minimum_wait_calls_(minimum_wait_calls),
                  waited_calls_(0) {
            previous_time_ = ros::Time::now();
            following_updater_ = std::move(following_updater);
        }

        bool Periodic::updateSegments(TrajectorySegment *root) {
            // Both conditions need to be met
            if ((ros::Time::now() - previous_time_).toSec() >= p_minimum_wait_time_ &&
                waited_calls_ >= p_minimum_wait_calls_) {
                previous_time_ = ros::Time::now();
                waited_calls_ = 0;
                return following_updater_->updateSegments(root);
            }
            waited_calls_++;
            return true;
        }

        void Periodic::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<double>(param_map, "minimum_wait_time", &p_minimum_wait_time_, 0.0);
            setParam<int>(param_map, "minimum_wait_calls", &p_minimum_wait_calls_, 0);
            waited_calls_ = 0;
            previous_time_ = ros::Time::now();

            // Create Following updater (default does nothing)
            std::string args;   // default args extends the parent namespace
            std::string param_ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "following_updater_args", &args, param_ns + "/following_updater");
            following_updater_ = ModuleFactory::Instance()->createEvaluatorUpdater(args, parent_, verbose_modules_);
        }

    } // namespace evaluator_updaters
} // namepsace mav_active_3d_planning
