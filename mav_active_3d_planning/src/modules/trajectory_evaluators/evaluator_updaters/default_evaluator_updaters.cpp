#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/module_factory.h"

#include <ros/param.h>

#include <vector>
#include <algorithm>
#include <random>

namespace mav_active_3d_planning {
    namespace evaluator_updaters {

        // Don't perform any specific update operations
        class Void : public EvaluatorUpdater {
        public:
            Void() {}

            bool updateSegments(TrajectorySegment &root) {
                return true;
            }
        };

        // Evaluate all existing trajectories from scratch
        class EvaluateFromScratch : public EvaluatorUpdater {
        public:
            EvaluateFromScratch(TrajectoryEvaluator* parent) : EvaluatorUpdater(parent) {}

            bool updateSegments(TrajectorySegment &root) {
                // recursively evaluate all segments from root to leaves (as in the planner)
                updateSingle(root);
                return true;
            }

            void updateSingle(TrajectorySegment &segment){
                parent_->computeGain(segment);
                parent_->computeCost(segment);
                parent_->computeValue(segment);
                for (int i = 0; i < segment.children.size(); ++i) {
                    updateSingle(*(segment.children[i]));
                }
            }
        };

        // Remove all segments that dont have a minimum value, can then call another updater (Decorator pattern)
        class PruneByValue : public EvaluatorUpdater {
        public:
            PruneByValue(std::string param_ns, TrajectoryEvaluator* parent) : EvaluatorUpdater(parent) {
                // Params
                ros::param::param<double>(param_ns + "/minimum_value", minimum_value_, 0.0);
                ros::param::param<bool>(param_ns + "/use_relative_values", use_relative_values_, false);
                ros::param::param<bool>(param_ns + "/include_subsequent", include_subsequent_, false);

                // Following Updater (Default is Void)
                following_updater_ = ModuleFactory::createEvaluatorUpdater(param_ns+"/following_updater", parent);
            }

            bool updateSegments(TrajectorySegment &root) {
                double absolute_minimum = minimum_value_;
                if (use_relative_values_){
                    // Scale to 0 at minimum, 1 at maximum
                    std::vector<TrajectorySegment*> segments;
                    if (include_subsequent_){
                        root.getTree(segments);
                        segments.erase(segments.begin());   // Exclude root itself
                    } else {
                        root.getChildren(segments);
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
                    for (int i = 0; i < root.children.size(); ++i) {
                        if (root.children[j]->value < absolute_minimum){
                            root.children.erase(root.children.begin() + j);
                        } else {
                            j++;
                        }
                    }
                }
                return following_updater_->updateSegments(root);
            }

            bool removeSingle(TrajectorySegment &segment, double min_value){
                if (segment.children.empty()){
                    return (segment.value < min_value);
                }
                int j = 0;
                for (int i = 0; i < segment.children.size(); ++i) {
                    if (removeSingle(*segment.children[j], min_value)) {
                        segment.children.erase(segment.children.begin() + j);
                    } else {
                        j++;
                    }
                }
                return (segment.children.empty() & segment.value < min_value);
            }

        protected:
            double minimum_value_;
            bool use_relative_values_;
            bool include_subsequent_;
            EvaluatorUpdater* following_updater_;
        };

    } // namespace evaluator_updaters
} // namepsace mav_active_3d_planning
