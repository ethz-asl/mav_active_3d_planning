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
            Greedy(std::string param_ns) {
                ros::param::param<bool>(param_ns + "/leaves_only", leaves_only_, false);    // Leaves or tree
            }

            TrajectorySegment* selectSegment(TrajectorySegment &root){
                std::vector<TrajectorySegment*> candidates;
                if (leaves_only_) {
                    root.getLeaves(candidates);
                } else {
                    root.getTree(candidates);
                }
                return *std::max_element(candidates.begin(), candidates.end(), TrajectorySegment::comparePtr);
            }

        protected:
            bool leaves_only_;
        };

        // Select certain segments weighted random, most configurable
        class RandomWeighted : public SegmentSelector {
        public:
            RandomWeighted(std::string param_ns) {
                ros::param::param<double>(param_ns + "/factor", factor_, 0.0);  // weighting, 0 for uniform
                ros::param::param<bool>(param_ns + "/leaves_only", leaves_only_, false);    // Leaves or tree
                ros::param::param<bool>(param_ns + "/revisit", revisit_, false);    // only unchecked segments
                ros::param::param<double>(param_ns + "/uniform_weight", uniform_weight_, 0.0);    // part of probability
                // mass that is distributed uniformly (to guarantee non-zero probability for all candidates)
            }

            TrajectorySegment* selectSegment(TrajectorySegment &root){
                std::vector<TrajectorySegment*> candidates;
                if (leaves_only_) {
                    root.getLeaves(candidates);
                } else {
                    root.getTree(candidates);
                }
                if (!revisit_){
                    candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                            [](const TrajectorySegment* segment) {return segment->tg_visited;}), candidates.end());
                }
                if (factor_ > 0.0 && (double)rand()/RAND_MAX >= uniform_weight_){
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
                            return candidates[i];
                        }
                    }
                }
                return candidates[rand() % candidates.size()];
            }

        protected:
            bool leaves_only_;
            bool revisit_;
            double factor_;
            double uniform_weight_;
        };

    } // namespace segment_selectors
} // namepsace mav_active_3d_planning
