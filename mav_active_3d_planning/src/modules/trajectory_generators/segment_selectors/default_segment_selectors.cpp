#include "mav_active_3d_planning/modules/trajectory_generators/segment_selectors/default_segment_selectors.h"
#include "mav_active_3d_planning/defaults.h"

#include <vector>
#include <algorithm>
#include <random>

namespace mav_active_3d_planning {
    namespace segment_selectors {

        // GreedySelector
        ModuleFactory::Registration<GreedySelector> GreedySelector::registration("GreedySelector");

        GreedySelector::GreedySelector(bool leaves_only) : leaves_only_(leaves_only) {}

        bool GreedySelector::selectSegment(TrajectorySegment **result, TrajectorySegment *root) {
            std::vector < TrajectorySegment * > candidates;
            if (leaves_only_) {
                root->getLeaves(&candidates);
            } else {
                root->getTree(&candidates);
            }
            *result = *std::max_element(candidates.begin(), candidates.end(), TrajectorySegment::comparePtr);
            return true;
        }

        void GreedySelector::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<bool>(param_map, "leaves_only", &leaves_only_, false);
        }

        // RandomWeighted
        ModuleFactory::Registration<RandomWeighted> RandomWeighted::registration("RandomWeighted");

        RandomWeighted::RandomWeighted(double factor, double uniform_probability, double leaf_probability, bool revisit)
                : factor_(factor),
                  uniform_probability_(uniform_probability),
                  leaf_probability_(leaf_probability),
                  revisit_(revisit) {
            assureParamsValid();
        }

        bool RandomWeighted::selectSegment(TrajectorySegment **result, TrajectorySegment *root) {
            // Get all candidates
            std::vector < TrajectorySegment * > candidates;
            if ((double) rand() / RAND_MAX <= leaf_probability_) {
                root->getLeaves(&candidates);
            } else {
                root->getTree(&candidates);
            }
            if (!revisit_) {
                candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                                                [](const TrajectorySegment *segment) { return segment->tg_visited; }),
                                 candidates.end());
            }
            if (candidates.empty()) {
                // exception catching
                *result = root;
                return false;
            }
            if (factor_ > 0.0 && (double) rand() / RAND_MAX > uniform_probability_) {
                // Weighted selection, cdf of every element's value ^ factor
                std::vector<double> values(candidates.size());
                double value_sum = 0.0;
                // shift to 0 to compensate for negative values
                double min_value = (*std::min_element(candidates.begin(), candidates.end(),
                                                      TrajectorySegment::comparePtr))->value;

                // cumulated probability density function
                for (int i = 0; i < candidates.size(); ++i) {
                    value_sum += pow((candidates[i]->value - min_value), factor_);
                    values[i] = value_sum;
                }
                double realization = (double) rand() / RAND_MAX * value_sum;
                for (int i = 0; i < candidates.size(); ++i) {
                    if (values[i] >= realization) {
                        *result = candidates[i];
                        return true;
                    }
                }
            }
            // uniform selection
            *result = candidates[rand() % candidates.size()];
            return true;
        }

        void RandomWeighted::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<double>(param_map, "factor", &factor_, 1.0);
            setParam<double>(param_map, "uniform_probability", &uniform_probability_, 0.0);
            setParam<double>(param_map, "leaf_probability", &leaf_probability_, 0.0);
            setParam<bool>(param_map, "revisit", &revisit_, false);
        }

        bool RandomWeighted::checkParamsValid(std::string *error_message) {
            if (uniform_probability_ > 1.0 || uniform_probability_ < 0.0) {
                *error_message = "uniform_probability expected in [0.0, 1.0]";
                return false;
            } else if (leaf_probability_ > 1.0 || leaf_probability_ < 0.0) {
                *error_message = "leaf_probability expected in [0.0, 1.0]";
                return false;
            }
            return true;
        }

    } // namespace segment_selectors
} // namepsace mav_active_3d_planning
