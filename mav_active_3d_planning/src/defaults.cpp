#include "mav_active_3d_planning/defaults.h"

namespace mav_active_3d_planning {
    namespace defaults {
        TrajectorySegment *selectRandomUniform(std::vector < TrajectorySegment * > &candidates){
            return candidates[rand()%candidates.size()];
        }

        TrajectorySegment *selectRandomWeighted(std::vector < TrajectorySegment * > &candidates) {
            std::vector<double> values(candidates.size());
            double value_sum = 0.0;
            double min_value = 0.0;     // to compensate for negative values
            for (int i = 0; i < candidates.size(); ++i) {
                value_sum += candidates[i]->value;
                min_value = std::min(min_value, candidates[i]->value);
                values[i] = value_sum;
            }
            value_sum -= (double) candidates.size() * min_value;
            double realization = (double) rand() / RAND_MAX * value_sum;
            for (int i = 0; i < candidates.size(); ++i) {
                if ((values[i] - (i+1) * min_value) >= realization) {
                    return candidates[i];
                }
            }
        }

        TrajectorySegment *selectRandomLeafUniform(TrajectorySegment &root) {
            if (root.children.empty()) { return &root; }
            std::vector < TrajectorySegment * > candidates;
            root.getLeaves(candidates);
            return selectRandomUniform(candidates);
        }

        TrajectorySegment* selectRandomLeafWeighted(TrajectorySegment &root){
            if (root.children.empty()) { return &root; }
            std::vector < TrajectorySegment * > candidates;
            root.getLeaves(candidates);
            return selectRandomWeighted(candidates);
        }

        TrajectorySegment *selectRandomSegmentUniform(TrajectorySegment &root) {
            if (root.children.empty()) { return &root; }
            std::vector < TrajectorySegment * > candidates;
            root.getTree(candidates);
            return selectRandomUniform(candidates);
        }

        TrajectorySegment *selectRandomSegmentWeighted(TrajectorySegment &root) {
            if (root.children.empty()) { return &root; }
            std::vector < TrajectorySegment * > candidates;
            root.getTree(candidates);
            return selectRandomWeighted(candidates);
        }

    } // namespace defaults
} // namepsace mav_active_3d_planning
