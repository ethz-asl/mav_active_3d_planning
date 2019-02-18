#include "mav_active_3d_planning/trajectory_evaluator.h"

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

        // Linear combination of overall accumulatedcost and gain
        class LinearAccumulated : public ValueComputer {
        public:
            LinearAccumulated(std::string param_ns) {
                ros::param::param<double>(param_ns + "/cost_weight", cost_weight_, 1.0);
                ros::param::param<double>(param_ns + "/gain_weight", gain_weight_, 1.0);
            }

            bool computeValue(TrajectorySegment &traj_in) {
                double cost = traj_in.cost;
                double gain = traj_in.gain;
                TrajectorySegment *current = traj_in.parent;
                while (current) {
                    cost += current->cost;
                    gain += current->gain;
                    current = current->parent;
                }
                traj_in.value = gain_weight_ * gain - cost_weight_ * cost;
                return true;
            }

        protected:
            double cost_weight_;
            double gain_weight_;
        };

    } // namespace value_computers
} // namepsace mav_active_3d_planning
