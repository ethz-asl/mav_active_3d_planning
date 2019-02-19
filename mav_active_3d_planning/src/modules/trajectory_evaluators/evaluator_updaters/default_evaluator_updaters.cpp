#include "mav_active_3d_planning/trajectory_evaluator.h"

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

    } // namespace evaluator_updaters
} // namepsace mav_active_3d_planning
