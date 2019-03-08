#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/module_factory.h"

#include <ros/time.h>

#include <vector>
#include <algorithm>
#include <random>
#include <memory>

namespace mav_active_3d_planning {
    namespace evaluator_updaters {

        // Don't perform any specific update operations
        class UpdateNothing : public EvaluatorUpdater {
        public:
            UpdateNothing() {}

            bool updateSegments(TrajectorySegment *root) {
                return true;
            }

        protected:
            friend ModuleFactory;

            void setupFromParamMap(Module::ParamMap *param_map) {}
        };

        // Discard all segments and start from scratch
        class ResetTree : public EvaluatorUpdater {
        public:
            ResetTree() {}

            bool updateSegments(TrajectorySegment *root) {
                root->children.clear();
                return true;
            }
        protected:
            friend ModuleFactory;

            void setupFromParamMap(Module::ParamMap *param_map){}
        };

        // Update gain/cost/value for the complete trajectory tree
        class UpdateAll : public EvaluatorUpdater {
        public:
            UpdateAll(bool update_gain, bool update_cost, bool update_value,
                      std::unique_ptr<EvaluatorUpdater> following_updater)
                : update_gain_(update_gain),
                  update_cost_(update_cost),
                  update_value_(update_value) {
                following_updater_ = std::move(following_updater);
            }

            bool updateSegments(TrajectorySegment *root) {
                // recursively update all segments from root to leaves (as in the planner)
                updateSingle(root);
                return following_updater_->updateSegments(root);
            }
        protected:
            friend ModuleFactory;

            UpdateAll() {}

            void setupFromParamMap(Module::ParamMap *param_map){
                setParam<bool>(param_map, "update_gain", &update_gain_, true);
                setParam<bool>(param_map, "update_cost", &update_cost_, true);
                setParam<bool>(param_map, "update_value", &update_value_, true);

                // Create Following updater (default does nothing)
                std::string args;   // default args extends the parent namespace
                std::string param_ns;
                setParam<std::string>(param_map, "param_namespace", &param_ns, "");
                setParam<std::string>(param_map, "following_updater_args", &args, param_ns + "/following_updater");
                following_updater_ = ModuleFactory::Instance()->createEvaluatorUpdater(args, parent_, verbose_modules_);
            }

            void updateSingle(TrajectorySegment *segment){
                if (update_gain_) { parent_->computeGain(segment); }
                if (update_cost_) { parent_->computeCost(segment); }
                if (update_value_) { parent_->computeValue(segment); }
                for (int i = 0; i < segment->children.size(); ++i) {
                    updateSingle(segment->children[i].get());
                }
            }

            // variables
            bool update_gain_;
            bool update_cost_;
            bool update_value_;
            std::unique_ptr<EvaluatorUpdater> following_updater_;
        };

        // Remove all segments that dont have a minimum value, can then call another updater
        class PruneByValue : public EvaluatorUpdater {
        public:
            PruneByValue(double minimum_value, bool use_relative_values, bool include_subsequent,
                         std::unique_ptr<EvaluatorUpdater> following_updater)
                         : minimum_value_(minimum_value),
                           use_relative_values_(use_relative_values),
                           include_subsequent_(include_subsequent) {
                following_updater_ = std::move(following_updater);
            }

            bool updateSegments(TrajectorySegment *root) {
                double absolute_minimum = minimum_value_;
                if (use_relative_values_){
                    // Scale to 0 at minimum, 1 at maximum
                    std::vector<TrajectorySegment*> segments;
                    if (include_subsequent_){
                        root->getTree(segments);
                        segments.erase(segments.begin());   // Exclude root itself
                    } else {
                        root->getChildren(segments);
                    }
                    double min_value = (*std::min_element(segments.begin(), segments.end(),
                                                   TrajectorySegment::comparePtr))->value;
                    double max_value = (*std::max_element(segments.begin(), segments.end(),
                                                   TrajectorySegment::comparePtr))->value;
                    absolute_minimum = minimum_value_ * (max_value - min_value) + min_value;
                }
                if (include_subsequent_){
                    // remove  recursively (exluding the root itself)
                    removeSingle(root, absolute_minimum);
                } else {
                    int j = 0;
                    for (int i = 0; i < root->children.size(); ++i) {
                        if (root->children[j]->value < absolute_minimum){
                            root->children.erase(root->children.begin() + j);
                        } else {
                            j++;
                        }
                    }
                }
                return following_updater_->updateSegments(root);
            }

        protected:
            friend ModuleFactory;

            PruneByValue() {}

            void setupFromParamMap(Module::ParamMap *param_map){
                setParam<double>(param_map, "minimum_value", &minimum_value_, 0.0);
                setParam<bool>(param_map, "use_relative_values", &use_relative_values_, false);
                setParam<bool>(param_map, "include_subsequent", &include_subsequent_, true);

                // Create Following updater (default does nothing)
                std::string args;   // default args extends the parent namespace
                std::string param_ns;
                setParam<std::string>(param_map, "param_namespace", &param_ns, "");
                setParam<std::string>(param_map, "following_updater_args", &args, param_ns + "/following_updater");
                following_updater_ = ModuleFactory::Instance()->createEvaluatorUpdater(args, parent_, verbose_modules_);
            }

            bool removeSingle(TrajectorySegment *segment, double min_value){
                if (segment->children.empty()){
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

            // variables
            double minimum_value_;
            bool use_relative_values_;
            bool include_subsequent_;
            std::unique_ptr<EvaluatorUpdater> following_updater_;
        };

        // Only periodically call another updater
        class Periodic : public EvaluatorUpdater {
        public:
            Periodic(double minimum_wait_time, int minimum_wait_calls,
                     std::unique_ptr<EvaluatorUpdater> following_updater)
                     : p_minimum_wait_time_(minimum_wait_time),
                       p_minimum_wait_calls_(minimum_wait_calls),
                       waited_calls_(0) {
                previous_time_ = ros::Time::now();
                following_updater_ = std::move(following_updater);
            }

            bool updateSegments(TrajectorySegment *root) {
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

        protected:
            friend ModuleFactory;

            Periodic() {}

            void setupFromParamMap(Module::ParamMap *param_map){
                setParam<double>(param_map, "minimum_wait_time", &p_minimum_wait_time_, 0.0);
                setParam<int>(param_map, "minimum_wait_calls", &p_minimum_wait_calls_, 0);
                waited_calls_ = 0;
                previous_time_ = ros::Time::now();

                // Create Following updater (default does nothing)
                std::string args;   // default args extends the parent namespace
                std::string param_ns;
                setParam<std::string>(param_map, "param_namespace", &param_ns, "");
                setParam<std::string>(param_map, "following_updater_args", &args, param_ns + "/following_updater");
                following_updater_ = ModuleFactory::Instance()->createEvaluatorUpdater(args, parent_, verbose_modules_);
            }

            // variables
            double p_minimum_wait_time_;
            int p_minimum_wait_calls_;
            ros::Time previous_time_;   // Need to use simulated time here
            int waited_calls_;
            std::unique_ptr<EvaluatorUpdater> following_updater_;
        };

    } // namespace evaluator_updaters
} // namepsace mav_active_3d_planning
