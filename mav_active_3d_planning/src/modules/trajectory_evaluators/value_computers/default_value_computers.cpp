#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/module_factory.h"

#include <ros/param.h>
#include <cmath>

namespace mav_active_3d_planning {
    namespace value_computers {

        // Linear combination of cost and gain
        class Linear : public ValueComputer {
        public:
            Linear(std::string param_ns) {
                ros::param::param<double>(param_ns + "/cost_weight", cost_weight_, 1.0);
                ros::param::param<double>(param_ns + "/gain_weight", gain_weight_, 1.0);
            }

            bool computeValue(TrajectorySegment &traj_in) {
                traj_in.value = gain_weight_ * traj_in.gain - cost_weight_ * traj_in.cost;
                return true;
            }

        protected:
            double cost_weight_;
            double gain_weight_;
        };

        // Discount the gain with an exponential term of the cost
        class ExponentialDiscount : public ValueComputer {
        public:
            ExponentialDiscount(std::string param_ns) {
                ros::param::param<double>(param_ns + "/cost_scale", cost_scale_, 1.0);
            }

            bool computeValue(TrajectorySegment &traj_in) {
                traj_in.value = traj_in.gain *  std::exp(-1.0 * cost_scale_ *traj_in.cost);
                return true;
            }

        protected:
            double cost_scale_;
        };

        // Accumulates the values up to the root, use another value computer for individual values (Decorator pattern)
        class Accumulate : public ValueComputer {
        public:
            Accumulate(std::string param_ns) {
                following_value_computer_ = ModuleFactory::createValueComputer(param_ns+"/following_value_computer");
            }

            bool computeValue(TrajectorySegment &traj_in) {
                following_value_computer_->computeValue(traj_in);
                TrajectorySegment *current = traj_in.parent;
                while (current) {
                    traj_in.value = traj_in.value + current->value;
                    current = current->parent;
                }
                return true;
            }

        protected:
            ValueComputer* following_value_computer_;
        };

    } // namespace value_computers
} // namepsace mav_active_3d_planning
