#ifndef MAV_ACTIVE_3D_PLANNING_EVALUATOR_UPDATERS_DEFAULT_EVALUATOR_UPDATERS_H
#define MAV_ACTIVE_3D_PLANNING_EVALUATOR_UPDATERS_DEFAULT_EVALUATOR_UPDATERS_H

#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/module_factory.h"
#include "mav_active_3d_planning/planner_node_class.h"

#include <ros/time.h>

#include <memory>

namespace mav_active_3d_planning {
    namespace evaluator_updaters {

        // Don't perform any specific update operations
        class EvaluatorUpdateNothing : public EvaluatorUpdater {
        public:
            // override virtual functions
            bool updateSegments(TrajectorySegment *root) { return true; }

        protected:
            friend ModuleFactory;

            EvaluatorUpdateNothing() {}

            void setupFromParamMap(Module::ParamMap *param_map) {}

            static ModuleFactory::Registration<EvaluatorUpdateNothing> registration;
        };

        // Discard all segments and start from scratch
        class EvaluatorResetTree : public EvaluatorUpdater {
        public:
            // override virtual functions
            bool updateSegments(TrajectorySegment *root);

        protected:
            friend ModuleFactory;

            EvaluatorResetTree() {}

            void setupFromParamMap(Module::ParamMap *param_map){}

            static ModuleFactory::Registration<EvaluatorResetTree> registration;
        };

        // Update gain/cost/value for the complete trajectory tree
        class UpdateAll : public EvaluatorUpdater {
        public:
            // override virtual functions
            bool updateSegments(TrajectorySegment *root);

        protected:
            friend ModuleFactory;

            // factory acces
            UpdateAll() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            static ModuleFactory::Registration<UpdateAll> registration;

            // methods
            void updateSingle(TrajectorySegment *segment);

            // params
            bool update_gain_;
            bool update_cost_;
            bool update_value_;
            std::unique_ptr<EvaluatorUpdater> following_updater_;

            // pointer to the planner node to call for updates
            PlannerNode* planner_node_;
        };

        // Remove all segments that dont have a minimum value, can then call another updater
        class PruneByValue : public EvaluatorUpdater {
        public:
            // override virtual functions
            bool updateSegments(TrajectorySegment *root);

        protected:
            friend ModuleFactory;

            // factory access
            PruneByValue() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            static ModuleFactory::Registration<PruneByValue> registration;

            // methods
            bool removeSingle(TrajectorySegment *segment, double min_value);
            double getMinimumValue(TrajectorySegment *segment);

            // params
            double minimum_value_;          // value below which segments are cropped
            bool use_relative_values_;      // if true scale all existing values to 0-1
            bool include_subsequent_;       // if true keep all segments where at least 1 child matches the min value
            std::unique_ptr<EvaluatorUpdater> following_updater_;
        };

        // Only periodically call another updater
        class UpdatePeriodic : public EvaluatorUpdater {
        public:

            // override virtual functions
            bool updateSegments(TrajectorySegment *root);

        protected:
            friend ModuleFactory;

            // factory access
            UpdatePeriodic() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            static ModuleFactory::Registration<UpdatePeriodic> registration;

            // params
            double p_minimum_wait_time_;
            int p_minimum_wait_calls_;

            // variables
            ros::Time previous_time_;   // Need to use simulated time here
            int waited_calls_;

            // members
            std::unique_ptr<EvaluatorUpdater> following_updater_;
        };

        // Update gain, only if conditions are met (min gain, nearby)
        class ConstrainedUpdater : public EvaluatorUpdater {
        public:
            // override virtual functions
            bool updateSegments(TrajectorySegment *root);

        protected:
            friend ModuleFactory;

            // factory acces
            ConstrainedUpdater() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            static ModuleFactory::Registration<ConstrainedUpdater> registration;

            // methods
            void updateSingle(TrajectorySegment *segment);

            // params
            double p_minimum_gain_;
            double p_update_range_;
            std::unique_ptr<EvaluatorUpdater> following_updater_;

            // pointer to the planner node to call for updates
            PlannerNode* planner_node_;
        };

    } // namespace evaluator_updaters
} // namepsace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_EVALUATOR_UPDATERS_DEFAULT_EVALUATOR_UPDATERS_H

