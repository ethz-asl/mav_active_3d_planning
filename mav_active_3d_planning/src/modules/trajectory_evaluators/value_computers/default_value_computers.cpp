#include "mav_active_3d_planning/trajectory_evaluator.h"

#include <ros/param.h>

namespace mav_active_3d_planning {
    namespace value_computers {

        // Linear combination of cost and gain
        class LinearValue : public ValueComputer {
        public:
            LinearValue(double cost_weight, double gain_weight)
                : cost_weight_(cost_weight),
                  gain_weight_(gain_weight) {}

            static std::unique_ptr<ValueComputer> createFromRos(const ros::NodeHandle &nh, bool verbose){
                if (verbose) {
                    S_INFO("Creating 'ValueComputer: LinearValue' from '%s' with parameters:", nh.getNamespace());
                }

                // Default values
                double cost_weight = 1.0;
                double gain_weight = 1.0;

                // Get parameters
                defaults::setParamFromRos<double>(nh, "cost_weight", &cost_weight, verbose);
                defaults::setParamFromRos<double>(nh, "gain_weight", &gain_weight, verbose, &LinearValue::validGain);

                return std::unique_ptr<ValueComputer>(new LinearValue(cost_weight, gain_weight));
            }

            static bool validGain(double gain){
                // For param requirements, test example
                return gain > 0.0;
            }

            bool computeValue(TrajectorySegment &traj_in) {
                traj_in.value = gain_weight_ * traj_in.gain - cost_weight_ * traj_in.cost;
                return true;
            }

        protected:
            double cost_weight_;
            double gain_weight_;
        };

        // This module is replaced in next version

    } // namespace value_computers
} // namepsace mav_active_3d_planning
