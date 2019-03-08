#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/module_factory.h"

#include <cmath>

namespace mav_active_3d_planning {
    namespace value_computers {

        // Linear combination of cost and gain
        class LinearValue : public ValueComputer {
        public:
            LinearValue(double cost_weight, double gain_weight)
                    : cost_weight_(cost_weight),
                      gain_weight_(gain_weight) {}

            bool computeValue(TrajectorySegment *traj_in) {
                traj_in->value = gain_weight_ * traj_in->gain - cost_weight_ * traj_in->cost;
                return true;
            }

        protected:
            friend ModuleFactory;

            LinearValue() {}

            void setupFromParamMap(Module::ParamMap *param_map) {
                setParam<double>(param_map, "cost_weight", &cost_weight_, 1.0);
                setParam<double>(param_map, "gain_weight", &gain_weight_, 1.0);
            }

            double cost_weight_;
            double gain_weight_;
        };

        // Discount the gain with an exponential term of the cost
        class ExponentialDiscount : public ValueComputer {
        public:
            ExponentialDiscount(double cost_scale) : cost_scale_(cost_scale) {}

            bool computeValue(TrajectorySegment *traj_in) {
                traj_in->value = traj_in->gain * std::exp(-1.0 * cost_scale_ * traj_in->cost);
                return true;
            }

        protected:
            friend ModuleFactory;

            ExponentialDiscount() {}

            void setupFromParamMap(Module::ParamMap *param_map) {
                setParam<double>(param_map, "cost_scale", &cost_scale_, 1.0);
            }

            double cost_scale_;
        };

        // Accumulates the values up to the root, use another value computer for individual values (Decorator pattern)
        class Accumulate : public ValueComputer {
        public:
            Accumulate(std::unique_ptr <ValueComputer> following_value_computer) {
                following_value_computer_ = std::move(following_value_computer);
            }

            bool computeValue(TrajectorySegment *traj_in) {
                following_value_computer_->computeValue(traj_in);
                if (traj_in->parent) {
                    traj_in->value += traj_in->parent->value;
                }
                return true;
            }

        protected:
            friend ModuleFactory;

            Accumulate() {}

            void setupFromParamMap(Module::ParamMap *param_map) {
                // Create Following value computer
                std::string args;   // default args extends the parent namespace
                std::string param_ns;
                setParam<std::string>(param_map, "param_namespace", &param_ns, "");
                setParam<std::string>(param_map, "following_value_computer_args", &args,
                                      param_ns + "/following_value_computer");
                following_value_computer_ = ModuleFactory::Instance()->createValueComputer(args, verbose_modules_);
            }

            std::unique_ptr <ValueComputer> following_value_computer_;
        };

    } // namespace value_computers
} // namepsace mav_active_3d_planning
