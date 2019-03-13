#ifndef MAV_ACTIVE_3D_PLANNING_EVALUATOR_UPDATERS_DEFAULT_EVALUATOR_UPDATERS_H
#define MAV_ACTIVE_3D_PLANNING_EVALUATOR_UPDATERS_DEFAULT_EVALUATOR_UPDATERS_H

#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/module_factory.h"

#include <ros/time.h>

#include <memory>

namespace mav_active_3d_planning {
    namespace evaluator_updaters {

        // Don't perform any specific update operations
        class UpdateNothing : public EvaluatorUpdater {
        public:
            UpdateNothing() {}

            // override virtual functions
            bool updateSegments(TrajectorySegment *root) { return true; }

        protected:
            friend ModuleFactory;

            void setupFromParamMap(Module::ParamMap *param_map) {}
        };

        // Discard all segments and start from scratch
        class ResetTree : public EvaluatorUpdater {
        public:
            ResetTree() {}

            // override virtual functions
            bool updateSegments(TrajectorySegment *root);

        protected:
            friend ModuleFactory;

            void setupFromParamMap(Module::ParamMap *param_map){}
        };

        // Update gain/cost/value for the complete trajectory tree
        class UpdateAll : public EvaluatorUpdater {
        public:
            UpdateAll(bool update_gain, bool update_cost, bool update_value,
                      std::unique_ptr<EvaluatorUpdater> following_updater);

            // override virtual functions
            bool updateSegments(TrajectorySegment *root);

        protected:
            friend ModuleFactory;

            // factory acces
            UpdateAll() {}
            void setupFromParamMap(Module::ParamMap *param_map);

            // methods
            void updateSingle(TrajectorySegment *segment);

            // params
            bool update_gain_;
            bool update_cost_;
            bool update_value_;
            std::unique_ptr<EvaluatorUpdater> following_updater_;
        };

        // Remove all segments that dont have a minimum value, can then call another updater
        class PruneByValue : public EvaluatorUpdater {
        public:
            PruneByValue(double minimum_value, bool use_relative_values, bool include_subsequent,
                         std::unique_ptr<EvaluatorUpdater> following_updater);

            // override virtual functions
            bool updateSegments(TrajectorySegment *root);

        protected:
            friend ModuleFactory;

            // factory access
            PruneByValue() {}
            void setupFromParamMap(Module::ParamMap *param_map);

            // methods
            bool removeSingle(TrajectorySegment *segment, double min_value);

            // params
            double minimum_value_;
            bool use_relative_values_;
            bool include_subsequent_;
            std::unique_ptr<EvaluatorUpdater> following_updater_;
        };

        // Only periodically call another updater
        class Periodic : public EvaluatorUpdater {
        public:
            Periodic(double minimum_wait_time, int minimum_wait_calls,
                     std::unique_ptr<EvaluatorUpdater> following_updater);

            // override virtual functions
            bool updateSegments(TrajectorySegment *root);

        protected:
            friend ModuleFactory;

            // factory access
            Periodic() {}
            void setupFromParamMap(Module::ParamMap *param_map);

            // params
            double p_minimum_wait_time_;
            int p_minimum_wait_calls_;

            // variables
            ros::Time previous_time_;   // Need to use simulated time here
            int waited_calls_;

            // members
            std::unique_ptr<EvaluatorUpdater> following_updater_;
        };

    } // namespace evaluator_updaters
} // namepsace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_EVALUATOR_UPDATERS_DEFAULT_EVALUATOR_UPDATERS_H

