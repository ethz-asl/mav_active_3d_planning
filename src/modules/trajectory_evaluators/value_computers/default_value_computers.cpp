#include "mav_active_3d_planning/modules/trajectory_evaluators/value_computers/default_value_computers.h"

#include <cmath>

namespace mav_active_3d_planning {
    namespace value_computers {

        // LinearValue
        ModuleFactory::Registration<LinearValue> LinearValue::registration("LinearValue");

        bool LinearValue::computeValue(TrajectorySegment *traj_in) {
            double gain = traj_in->gain;
            double cost = traj_in->cost;
            if (p_accumulate_cost_){
                // accumulate cost up to the root
                TrajectorySegment* current = traj_in->parent;
                while (current) {
                    cost += current->cost;
                    current = current->parent;
                }
            }
            if (p_accumulate_gain_){
                // accumulate gain up to the root
                TrajectorySegment* current = traj_in->parent;
                while (current) {
                    gain += current->gain;
                    current = current->parent;
                }
            }
            traj_in->value = gain_weight_ * gain - cost_weight_ * cost;
            return true;
        }

        void LinearValue::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<double>(param_map, "cost_weight", &cost_weight_, 1.0);
            setParam<double>(param_map, "gain_weight", &gain_weight_, 1.0);
            setParam<bool>(param_map, "accumulate_cost", &p_accumulate_cost_, false);
            setParam<bool>(param_map, "accumulate_gain", &p_accumulate_gain_, false);
        }

        // ExponentialDiscount
        ModuleFactory::Registration<ExponentialDiscount> ExponentialDiscount::registration("ExponentialDiscount");

        bool ExponentialDiscount::computeValue(TrajectorySegment *traj_in) {
            double gain = traj_in->gain;
            double cost = traj_in->cost;
            if (p_accumulate_cost_){
                // accumulate cost up to the root
                TrajectorySegment* current = traj_in->parent;
                while (current) {
                    cost += current->cost;
                    current = current->parent;
                }
            }
            if (p_accumulate_gain_){
                // accumulate gain up to the root
                TrajectorySegment* current = traj_in->parent;
                while (current) {
                    gain += current->gain;
                    current = current->parent;
                }
            }
            traj_in->value = gain * std::exp(-1.0 * p_cost_scale_ * cost);
            return true;
        }

        void ExponentialDiscount::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<double>(param_map, "cost_scale", &p_cost_scale_, 1.0);
            setParam<bool>(param_map, "accumulate_cost", &p_accumulate_cost_, false);
            setParam<bool>(param_map, "accumulate_gain", &p_accumulate_gain_, false);
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

        // DiscountedRelativeGain
        ModuleFactory::Registration<DiscountedRelativeGain> DiscountedRelativeGain::registration("DiscountedRelativeGain");

        bool DiscountedRelativeGain::computeValue(TrajectorySegment *traj_in) {
            double gain = 0.0;
            double cost = 0.0;
            double factor = 1.0;
            iterate(traj_in, &factor, &gain, &cost);
            if (cost == 0.0) {
                traj_in->value = 0.0;
            } else {
                traj_in->value = gain / cost;
            }
            return true;
        }

        void DiscountedRelativeGain::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<double>(param_map, "discount_factor", &p_discount_factor_, 0.9);
        }

        void DiscountedRelativeGain::iterate(TrajectorySegment *current, double *factor, double *gain, double *cost){
            // iterate up to root
            if (current->parent) {
                iterate(current->parent, factor, gain, cost);
            }
            *gain += *factor * current->gain;
            *cost += current->cost;
            *factor *= p_discount_factor_;
        }

        // TrivialGain
        ModuleFactory::Registration<TrivialGain> TrivialGain::registration("TrivialGain");

        bool TrivialGain::computeValue(TrajectorySegment *traj_in) {
            traj_in->value = traj_in->gain;
            return true;
        }

        void TrivialGain::setupFromParamMap(Module::ParamMap *param_map) {
            // Create Following value computer
            std::string args;   // default args extends the parent namespace
            std::string param_ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "following_value_computer_args", &args,
                                  param_ns + "/following_value_computer");
        }

    } // namespace value_computers
} // namepsace mav_active_3d_planning
