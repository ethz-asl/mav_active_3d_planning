#include "mav_active_3d_planning/module_factory.h"

#include <ros/param.h>
#include <ros/console.h>

// Include available modules. Modules are to be included and created ONLY though the factory
#include "modules/trajectory_generators/uniform.cpp"
#include "modules/trajectory_generators/random_linear.cpp"
#include "modules/trajectory_generators/mav_trajectory_generation.cpp"
#include "modules/trajectory_evaluators/naive.cpp"
#include "modules/trajectory_evaluators/frontier.cpp"
#include "modules/segment_selectors/default_segment_selectors.cpp"
#include "modules/cost_computers/default_cost_computers.cpp"
#include "modules/value_computers/default_value_computers.cpp"
#include "modules/next_selectors/default_next_selectors.cpp"
#include "modules/back_trackers/default_back_trackers.cpp"


namespace mav_active_3d_planning {

    TrajectoryGenerator *ModuleFactory::createTrajectoryGenerator(voxblox::EsdfServer *voxblox_ptr,
                                                                  std::string param_ns) {
        std::string type;
        ros::param::param<std::string>(param_ns + "/type", type, "Uniform");
        std::string test = param_ns + "/type";
        if (type == "Uniform") {
            return new trajectory_generators::Uniform(voxblox_ptr, param_ns);
        } else if (type == "RandomLinear") {
            return new trajectory_generators::RandomLinear(voxblox_ptr, param_ns);
        } else if (type == "MavTrajectoryGeneration") {
            return new trajectory_generators::MavTrajectoryGeneration(voxblox_ptr, param_ns);
        } else {
            ROS_ERROR("Unknown TrajectoryGenerator type '%s'.", type.c_str());
            return nullptr;
        }
    }

    TrajectoryEvaluator *ModuleFactory::createTrajectoryEvaluator(voxblox::EsdfServer *voxblox_ptr,
                                                                  std::string param_ns) {
        std::string type;
        ros::param::param<std::string>(param_ns + "/type", type, "Naive");
        if (type == "Naive") {
            return new trajectory_evaluators::Naive(voxblox_ptr, param_ns);
        } else if (type == "Frontier") {
            return new trajectory_evaluators::Frontier(voxblox_ptr, param_ns);
        } else {
            ROS_ERROR("Unknown TrajectoryEvaluator type '%s'.", type.c_str());
            return nullptr;
        }
    }

    SegmentSelector *ModuleFactory::createSegmentSelector(std::string param_ns) {
        std::string type;
        ros::param::param<std::string>(param_ns + "/type", type, "Greedy");
        if (type == "Greedy") {
            return new segment_selectors::Greedy(param_ns);
        } else if (type == "RandomWeighted") {
            return new segment_selectors::RandomWeighted(param_ns);
        } else {
            ROS_ERROR("Unknown SegmentSelector type '%s'.", type.c_str());
            return nullptr;
        }
    }

    CostComputer *ModuleFactory::createCostComputer(std::string param_ns) {
        std::string type;
        ros::param::param<std::string>(param_ns + "/type", type, "Time");
        if (type == "Time") {
            return new cost_computers::Time();
        } else if (type == "Distance") {
            return new cost_computers::Distance();
        } else {
            ROS_ERROR("Unknown CostComputer type '%s'.", type.c_str());
            return nullptr;
        }
    }

    ValueComputer *ModuleFactory::createValueComputer(std::string param_ns){
        std::string type;
        ros::param::param<std::string>(param_ns + "/type", type, "Linear");
        if (type == "Linear") {
            return new value_computers::Linear(param_ns); }
        else if (type == "LinearAccumulated") {
            return new value_computers::LinearAccumulated(param_ns); }
        else {
            ROS_ERROR("Unknown ValueComputer type '%s'.", type.c_str());
            return nullptr;
        }
    }

    NextSelector *ModuleFactory::createNextSelector(std::string param_ns) {
        std::string type;
        ros::param::param<std::string>(param_ns + "/type", type, "ImmediateBest");
        if (type == "ImmediateBest") {
            return new next_selectors::ImmediateBest();
        } else if (type == "SubsequentBest") {
            return new next_selectors::SubsequentBest();
        } else {
            ROS_ERROR("Unknown NextSelector type '%s'.", type.c_str());
            return nullptr;
        }
    }

    BackTracker *ModuleFactory::createBackTracker(std::string param_ns) {
        std::string type;
        ros::param::param<std::string>(param_ns + "/type", type, "Rotate");
        if (type == "Rotate") {
            return new back_trackers::Rotate(param_ns);
        } else if (type == "RotateReverse") {
            return new back_trackers::RotateReverse(param_ns);
        } else {
            ROS_ERROR("Unknown BackTracker type '%s'.", type.c_str());
            return nullptr;
        }
    }

} // namepsace mav_active_3d_planning