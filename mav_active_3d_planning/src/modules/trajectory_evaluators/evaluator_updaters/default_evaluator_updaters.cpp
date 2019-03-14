#include "mav_active_3d_planning/trajectory_evaluator.h"

#include <vector>
#include <algorithm>
#include <random>

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

            void setupFromParamMap(ParamMap *param_map) {}
        };

    } // namespace evaluator_updaters
} // namepsace mav_active_3d_planning
