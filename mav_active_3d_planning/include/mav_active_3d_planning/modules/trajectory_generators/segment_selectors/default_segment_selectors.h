#ifndef MAV_ACTIVE_3D_PLANNING_SEGMENT_SELECTORS_DEFAULT_SEGMENT_SELECTORS_H
#define MAV_ACTIVE_3D_PLANNING_SEGMENT_SELECTORS_DEFAULT_SEGMENT_SELECTORS_H

#include "mav_active_3d_planning/trajectory_generator.h"

namespace mav_active_3d_planning {
    class ModuleFactory;

    namespace segment_selectors {

        // Select segment with highest value
        class Greedy : public SegmentSelector {
        public:
            Greedy(bool leaves_only);

            // override virtual functions
            bool selectSegment(TrajectorySegment **result, TrajectorySegment *root);

        protected:
            friend ModuleFactory;

            // factory access
            Greedy() {}
            void setupFromParamMap(Module::ParamMap *param_map);

            // variables
            bool leaves_only_;
        };

        // Select segments at random, weighted with their value
        class RandomWeighted : public SegmentSelector {
        public:
            RandomWeighted(double factor, double uniform_probability, double leaf_probability, bool revisit);

            // override virtual functions

            bool selectSegment(TrajectorySegment **result, TrajectorySegment *root);

        protected:
            friend ModuleFactory;

            // factory access
            RandomWeighted() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            bool checkParamsValid(std::string *error_message);

            // params
            double leaf_probability_;    // Probability to select a leaf (set to 1 for leaves only)
            bool revisit_;  // only unchecked segments
            double factor_; // weighting (potency), 0 for uniform
            double uniform_probability_;// part of probability mass that is distributed uniformly (to guarantee non-zero
            // probability for all candidates)
        };

    } // namespace segment_selectors
} // namepsace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_SEGMENT_SELECTORS_DEFAULT_SEGMENT_SELECTORS_H

