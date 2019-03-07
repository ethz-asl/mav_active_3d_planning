#include "mav_active_3d_planning/trajectory_generator.h"
#include "mav_active_3d_planning/defaults.h"

#include <vector>
#include <algorithm>
#include <random>

namespace mav_active_3d_planning {
    namespace segment_selectors {

        // Select segment with highest value
        class Greedy : public SegmentSelector {
        public:
            Greedy(bool leaves_only) : leaves_only_(leaves_only) {}

            bool selectSegment(TrajectorySegment *result, TrajectorySegment *root) {
                std::vector<TrajectorySegment*> candidates;
                if (leaves_only_) {
                    root->getLeaves(candidates);
                } else {
                    root->getTree(candidates);
                }
                result = *std::max_element(candidates.begin(), candidates.end(), TrajectorySegment::comparePtr);
                return true;
            }

        protected:
            friend ModuleFactory;

            Greedy() {}

            void setupFromParamMap(ParamMap *param_map){
                setParam<bool>(param_map, "leaves_only", &leaves_only_, false);
            }

            // variables
            bool leaves_only_;
        };

        // Select segments at random, weighted with their value
        class RandomWeighted : public SegmentSelector {
        public:
            RandomWeighted(double factor, double uniform_weight, bool leaves_only, bool revisit)
                : factor_(factor),
                  uniform_weight_(uniform_weight),
                  revisit_(revisit),
                  leaves_only_(leaves_only){
            }

            bool selectSegment(TrajectorySegment *result, TrajectorySegment *root) {
                std::vector<TrajectorySegment*> candidates;
                // Get all candidates
                if (leaves_only_) {
                    root->getLeaves(candidates);
                } else {
                    root->getTree(candidates);
                }
                if (!revisit_){
                    candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                            [](const TrajectorySegment* segment) {return segment->tg_visited;}), candidates.end());
                }
                if (factor_ > 0.0 && (double)rand()/RAND_MAX >= uniform_weight_){
                    // Weighted selection, cdf of every elements value ^ factor
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
                            result = candidates[i];
                            return true;
                        }
                    }
                }
                // uniform selection
                result = candidates[rand() % candidates.size()];
                return true;
            }

        protected:
            friend ModuleFactory;

            RandomWeighted() {}

            void setupFromParamMap(ParamMap *param_map){
                setParam<double>(param_map, "factor", &factor_, 0.0);
                setParam<double>(param_map, "uniform_weight", &uniform_weight_, 0.0);
                setParam<bool>(param_map, "leaves_only", &leaves_only_, false);
                setParam<bool>(param_map, "revisit", &revisit_, false);
            }

            // variables
            bool leaves_only_;
            bool revisit_;  // only unchecked segments
            double factor_; // weighting (potency), 0 for uniform
            double uniform_weight_;// part of probability mass that is distributed uniformly (to guarantee non-zero
            // probability for all candidates)
        };

    } // namespace segment_selectors
} // namepsace mav_active_3d_planning
