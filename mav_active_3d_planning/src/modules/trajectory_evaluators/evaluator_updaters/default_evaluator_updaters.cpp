#include "mav_active_3d_planning/modules/trajectory_evaluators/evaluator_updaters/default_evaluator_updaters.h"
#include "mav_active_3d_planning/planner_node.h"

#include <vector>
#include <algorithm>
#include <random>

namespace mav_active_3d_planning {
    namespace evaluator_updaters {

        ModuleFactory::Registration<EvaluatorUpdateNothing> EvaluatorUpdateNothing::registration("EvaluatorUpdateNothing");

        // EvaluatorResetTree
        ModuleFactory::Registration<EvaluatorResetTree> EvaluatorResetTree::registration("EvaluatorResetTree");

        bool EvaluatorResetTree::updateSegments(TrajectorySegment *root) {
            root->children.clear();
            return true;
        }

        // UpdateAll
        ModuleFactory::Registration<UpdateAll> UpdateAll::registration("UpdateAll");

        bool UpdateAll::updateSegments(TrajectorySegment *root) {
            // recursively update all segments from root to leaves (as in the planner)
            for (int i = 0; i < root->children.size(); ++i) {
                updateSingle(root->children[i].get());
            }
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
            following_updater_ = ModuleFactory::Instance()->createModule<EvaluatorUpdater>(args, verbose_modules_);
            planner_node_ = dynamic_cast<PlannerNode *>(ModuleFactory::Instance()->readLinkableModule("PlannerNode"));
        }

        void UpdateAll::updateSingle(TrajectorySegment *segment) {
            if (update_gain_) { planner_node_->trajectory_evaluator_->computeGain(segment); }
            if (update_cost_) { planner_node_->trajectory_evaluator_->computeCost(segment); }
            if (update_value_) { planner_node_->trajectory_evaluator_->computeValue(segment); }
            for (int i = 0; i < segment->children.size(); ++i) {
                updateSingle(segment->children[i].get());
            }
        }

        // PruneByValue
        ModuleFactory::Registration<PruneByValue> PruneByValue::registration("PruneByValue");

        void PruneByValue::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<double>(param_map, "minimum_value", &minimum_value_, 0.0);
            setParam<bool>(param_map, "use_relative_values", &use_relative_values_, false);
            setParam<bool>(param_map, "include_subsequent", &include_subsequent_, true);

            // Create Following updater (default does nothing)
            std::string args;   // default args extends the parent namespace
            std::string param_ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "following_updater_args", &args, param_ns + "/following_updater");
            following_updater_ = ModuleFactory::Instance()->createModule<EvaluatorUpdater>(args, verbose_modules_);
        }

        bool PruneByValue::updateSegments(TrajectorySegment *root) {
            // Remove all segments, whose value is below the minimum value
            double absolute_minimum = getMinimumValue(root);
            if (include_subsequent_) {
                // remove recursively, all segments where no child is above the minimum
                removeSingle(root, absolute_minimum);
            } else {
                // Remove the immediate children that are below the minimum
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

        double PruneByValue::getMinimumValue(TrajectorySegment *segment) {
            if (use_relative_values_) {
                // Scale from 0 at minimum, 1 at maximum to absolute minimum
                std::vector < TrajectorySegment * > segments;
                if (include_subsequent_) {
                    // Get the full tree without the root (which has always 0 value)
                    segment->getTree(&segments);
                    segments.erase(segments.begin());
                } else {
                    segment->getChildren(&segments);
                }
                double min_value = (*std::min_element(segments.begin(), segments.end(),
                                                      TrajectorySegment::comparePtr))->value;
                double max_value = (*std::max_element(segments.begin(), segments.end(),
                                                      TrajectorySegment::comparePtr))->value;
                return minimum_value_ * (max_value - min_value) + min_value;
            } else {
                // The absolute minimum is explicitely set
                return minimum_value_;
            }
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

        // UpdatePeriodic
        ModuleFactory::Registration<UpdatePeriodic> UpdatePeriodic::registration("UpdatePeriodic");

        bool UpdatePeriodic::updateSegments(TrajectorySegment *root) {
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

        void UpdatePeriodic::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<double>(param_map, "minimum_wait_time", &p_minimum_wait_time_, 0.0);
            setParam<int>(param_map, "minimum_wait_calls", &p_minimum_wait_calls_, 0);
            waited_calls_ = 0;
            previous_time_ = ros::Time::now();

            // Create Following updater (default does nothing)
            std::string args;   // default args extends the parent namespace
            std::string param_ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "following_updater_args", &args, param_ns + "/following_updater");
            following_updater_ = ModuleFactory::Instance()->createModule<EvaluatorUpdater>(args, verbose_modules_);
        }

    } // namespace evaluator_updaters
} // namepsace mav_active_3d_planning
