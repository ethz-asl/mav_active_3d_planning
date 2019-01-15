#include "mav_active_3d_planning/defaults.h"

#include <random>
#include <memory>


namespace mav_active_3d_planning {
    namespace defaults {
        // Select a random leaf of the trajectory tree
        TrajectorySegment *selectRandomLeaf(TrajectorySegment &root) {
            if (root.children.empty()) { return &root; }
            std::vector < TrajectorySegment * > candidates;
            root.getLeaves(candidates);
            srand(time(NULL));
            return candidates[rand()%candidates.size()];
        }

        // Select a random leaf of the trajectory tree, every leaf is weighted with its value
        TrajectorySegment* selectRandomLeafWeighted(TrajectorySegment &root, double uniform_weight){
            if (root.children.empty()) { return &root; }
            std::vector < TrajectorySegment * > candidates;
            root.getLeaves(candidates);

            // probability to select uniform random (to give a minimum non-zero probability to all leaves)
            srand(time(NULL));
            if ((double) rand() / RAND_MAX < uniform_weight){
                return candidates[rand()%candidates.size()];
            }

            // select weighted with value
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
            return &root; // Exception catching, should never happen by construction
        }

    } // namespace defaults
} // namepsace mav_active_3d_planning
