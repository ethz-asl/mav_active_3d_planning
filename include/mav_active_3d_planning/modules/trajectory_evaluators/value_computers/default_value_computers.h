#ifndef MAV_ACTIVE_3D_PLANNING_VALUE_COMPUTERS_DEFAULT_VALUE_COMPUTERS_H
#define MAV_ACTIVE_3D_PLANNING_VALUE_COMPUTERS_DEFAULT_VALUE_COMPUTERS_H

#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/module_factory.h"


namespace mav_active_3d_planning {
    namespace value_computers {

        // Linear combination of cost and gain
        class LinearValue : public ValueComputer {
        public:
            // override virtual functions
            bool computeValue(TrajectorySegment *traj_in);

        protected:
            friend ModuleFactory;

            // factory access
            LinearValue() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            static ModuleFactory::Registration<LinearValue> registration;

            // params
            double cost_weight_;
            double gain_weight_;
            bool p_accumulate_cost_;     // If true first accumulate all cost, then discount
            bool p_accumulate_gain_;     // If true first accumulate all cost, then discount
        };

        // Discount the gain with an exponential term of the cost
        class ExponentialDiscount : public ValueComputer {
        public:
            // override virtual functions
            bool computeValue(TrajectorySegment *traj_in);

        protected:
            friend ModuleFactory;

            // factory access
            ExponentialDiscount() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            static ModuleFactory::Registration<ExponentialDiscount> registration;

            // params
            double p_cost_scale_;
            bool p_accumulate_cost_;     // If true first accumulate all cost, then discount
            bool p_accumulate_gain_;     // If true first accumulate all gain, then discount

        };

        // Accumulates the values up to the root, use another value computer for individual values (Decorator pattern)
        class AccumulateValue : public ValueComputer {
        public:
            // override virtual functions
            bool computeValue(TrajectorySegment *traj_in);

        protected:
            friend ModuleFactory;

            // factory access
            AccumulateValue() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            static ModuleFactory::Registration<AccumulateValue> registration;

            // members
            std::unique_ptr <ValueComputer> following_value_computer_;
        };

        // Computes efficiency as gain divided by cost
        class RelativeGain : public ValueComputer {
        public:
            // override virtual functions
            bool computeValue(TrajectorySegment *traj_in);

        protected:
            friend ModuleFactory;

            // factory access
            RelativeGain() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            static ModuleFactory::Registration<RelativeGain> registration;

            // params
            bool p_accumulate_;     // true: return total gain divided by total cost
        };

        // Computes efficiency as gain divided by cost, where future gains are discounted per segment
        class DiscountedRelativeGain : public ValueComputer {
        public:
            // override virtual functions
            bool computeValue(TrajectorySegment *traj_in);

        protected:
            friend ModuleFactory;

            // factory access
            DiscountedRelativeGain() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            static ModuleFactory::Registration<DiscountedRelativeGain> registration;

            // params
            double p_discount_factor_;

            // recursively compute the discounted gain sum
            void iterate(TrajectorySegment *current, double *factor, double *gain, double *cost);
        };

        // Use the best subsequent value for each segment in the subtree
        class GlobalNormalizedGain : public ValueComputer {
        public:
            // override virtual functions
            bool computeValue(TrajectorySegment *traj_in);

        protected:
            friend ModuleFactory;

            // factory access
            GlobalNormalizedGain() {}
            static ModuleFactory::Registration<GlobalNormalizedGain> registration;
            void setupFromParamMap(Module::ParamMap *param_map) {}


            // methods
            double findBest(TrajectorySegment *current, double gain, double cost);
        };

    } // namespace value_computers
} // namepsace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_VALUE_COMPUTERS_DEFAULT_VALUE_COMPUTERS_H

