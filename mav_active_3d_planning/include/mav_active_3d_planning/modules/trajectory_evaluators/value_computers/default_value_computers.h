#ifndef MAV_ACTIVE_3D_PLANNING_VALUE_COMPUTERS_DEFAULT_VALUE_COMPUTERS_H
#define MAV_ACTIVE_3D_PLANNING_VALUE_COMPUTERS_DEFAULT_VALUE_COMPUTERS_H

#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/module_factory.h"


namespace mav_active_3d_planning {
    namespace value_computers {

        // Linear combination of cost and gain
        class LinearValue : public ValueComputer {
        public:
            LinearValue(double cost_weight, double gain_weight);

            // override virtual functions
            bool computeValue(TrajectorySegment *traj_in);

        protected:
            friend ModuleFactory;

            // factory access
            LinearValue() {}
            void setupFromParamMap(Module::ParamMap *param_map);

            // params
            double cost_weight_;
            double gain_weight_;
        };

        // Discount the gain with an exponential term of the cost
        class ExponentialDiscount : public ValueComputer {
        public:
            ExponentialDiscount(double cost_scale);

            // override virtual functions
            bool computeValue(TrajectorySegment *traj_in);

        protected:
            friend ModuleFactory;

            // factory access
            ExponentialDiscount() {}
            void setupFromParamMap(Module::ParamMap *param_map);

            // params
            double cost_scale_;
        };

        // Accumulates the values up to the root, use another value computer for individual values (Decorator pattern)
        class Accumulate : public ValueComputer {
        public:
            Accumulate(std::unique_ptr <ValueComputer> following_value_computer);

            // override virtual functions
            bool computeValue(TrajectorySegment *traj_in);

        protected:
            friend ModuleFactory;

            // factory access
            Accumulate() {}
            void setupFromParamMap(Module::ParamMap *param_map);

            // params
            std::unique_ptr <ValueComputer> following_value_computer_;
        };

    } // namespace value_computers
} // namepsace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_VALUE_COMPUTERS_DEFAULT_VALUE_COMPUTERS_H

