#include "mav_active_3d_planning/modules/trajectory_evaluators/next_selectors/default_next_selectors.h"

#include <algorithm>
#include <vector>

namespace mav_active_3d_planning {
    namespace next_selectors {

        // ImmediateBest
        int ImmediateBest::selectNextBest(const TrajectorySegment &traj_in) {
            std::vector<int> candidates = {0};
            double current_max = traj_in.children[0]->value;
            for (int i = 1; i < traj_in.children.size(); ++i){
                if (traj_in.children[i]->value > current_max){
                    current_max = traj_in.children[i]->value;
                    candidates.clear();
                    candidates.push_back(i);
                } else if (traj_in.children[i]->value == current_max){
                    candidates.push_back(i);
                }
            }
            // randomize if multiple maxima
            std::random_shuffle(candidates.begin(), candidates.end());
            return candidates[0];
        }

        // SubsequentBest
        int SubsequentBest::selectNextBest(const TrajectorySegment &traj_in) {
            std::vector<int> candidates = {0};
            double current_max = evaluateSingle(traj_in.children[0].get());
            for (int i = 1; i < traj_in.children.size(); ++i){
                double current_value = evaluateSingle(traj_in.children[i].get());
                if (current_value > current_max){
                    current_max = current_value;
                    candidates.clear();
                    candidates.push_back(i);
                } else if (current_value == current_max){
                    candidates.push_back(i);
                }
            }
            // randomize if multiple maxima
            std::random_shuffle(candidates.begin(), candidates.end());
            return candidates[0];
        }

        double SubsequentBest::evaluateSingle(TrajectorySegment *traj_in) {
            //Recursively find highest value
            if (traj_in->children.empty()) {
                return traj_in->value;
            }
            double highest_value = traj_in->value;
            for (int i = 0; i < traj_in->children.size(); ++i) {
                highest_value = std::max(highest_value, evaluateSingle(traj_in->children[i].get()));
            }
            return highest_value;
        }

    } // namespace next_selectors
} // namepsace mav_active_3d_planning
