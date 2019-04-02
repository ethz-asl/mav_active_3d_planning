#include "mav_active_3d_planning/modules/trajectory_evaluators/value_computers/default_value_computers.h"

#include <cmath>

namespace mav_active_3d_planning {
    namespace value_computers {

        // LinearValue
        ModuleFactory::Registration<LinearValue> LinearValue::registration("LinearValue");

        bool LinearValue::computeValue(TrajectorySegment *traj_in) {
            traj_in->value = gain_weight_ * traj_in->gain - cost_weight_ * traj_in->cost;
            return true;
        }

        void LinearValue::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<double>(param_map, "cost_weight", &cost_weight_, 1.0);
            setParam<double>(param_map, "gain_weight", &gain_weight_, 1.0);
        }

        // ExponentialDiscount
        ModuleFactory::Registration<ExponentialDiscount> ExponentialDiscount::registration("ExponentialDiscount");

        bool ExponentialDiscount::computeValue(TrajectorySegment *traj_in) {
            traj_in->value = traj_in->gain * std::exp(-1.0 * cost_scale_ * traj_in->cost);
            return true;
        }

        void ExponentialDiscount::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<double>(param_map, "cost_scale", &cost_scale_, 1.0);
        }

        // AccumulateValue
        ModuleFactory::Registration<AccumulateValue> AccumulateValue::registration("AccumulateValue");

        bool AccumulateValue::computeValue(TrajectorySegment *traj_in) {
            following_value_computer_->computeValue(traj_in);
            if (traj_in->parent) {
                traj_in->value += traj_in->parent->value;
            }
            return true;
        }

        void AccumulateValue::setupFromParamMap(Module::ParamMap *param_map) {
            // Create Following value computer
            std::string args;   // default args extends the parent namespace
            std::string param_ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "following_value_computer_args", &args,
                                  param_ns + "/following_value_computer");
            following_value_computer_ = ModuleFactory::Instance()->createModule<ValueComputer>(args, verbose_modules_);
        }

        // RelativeGain
        ModuleFactory::Registration<RelativeGain> RelativeGain::registration("RelativeGain");

        bool RelativeGain::computeValue(TrajectorySegment *traj_in) {
            double gain = traj_in->gain;
            double cost = traj_in->cost;
            if (p_accumulate_) {
                TrajectorySegment *current = traj_in->parent;
                while (current) {
                    gain += current->gain;
                    cost += current->cost;
                    current = current->parent;
                }
            }
            if (cost == 0.0) {
                traj_in->value = 0.0;
            } else {
                traj_in->value = gain / cost;
            }
            return true;
        }

        void RelativeGain::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<bool>(param_map, "accumulate", &p_accumulate_, true);
        }

    } // namespace value_computers
} // namepsace mav_active_3d_planning
