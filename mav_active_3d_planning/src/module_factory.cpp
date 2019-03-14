#include "mav_active_3d_planning/module_factory.h"
#include "mav_active_3d_planning/defaults.h"

#include <ros/param.h>
#include <ros/node_handle.h>
#include <ros/console.h>

// Default modules
#include "modules/trajectory_generators/segment_selectors/default_segment_selectors.cpp"
#include "modules/trajectory_generators/generator_updaters/default_generator_updaters.cpp"
#include "modules/trajectory_evaluators/cost_computers/default_cost_computers.cpp"
#include "modules/trajectory_evaluators/value_computers/default_value_computers.cpp"
#include "modules/trajectory_evaluators/next_selectors/default_next_selectors.cpp"
#include "modules/trajectory_evaluators/evaluator_updaters/default_evaluator_updaters.cpp"
#include "modules/back_trackers/default_back_trackers.cpp"

// Include available modules. Modules are to be included and created ONLY though the factory
#include "modules/trajectory_generators/uniform.cpp"
#include "modules/trajectory_generators/random_linear.cpp"
#include "modules/trajectory_generators/mav_trajectory_generation.cpp"
#include "modules/trajectory_evaluators/naive.cpp"
#include "modules/trajectory_evaluators/frontier.cpp"


namespace mav_active_3d_planning {

    // Module lists
    TrajectoryGenerator *ModuleFactory::parseTrajectoryGenerators(std::string type) {
        if (type == "Uniform") {
            return new trajectory_generators::Uniform();
        } else if (type == "RandomLinear") {
            return new trajectory_generators::RandomLinear();
        } else if (type == "MavTrajectoryGeneration") {
            return new trajectory_generators::MavTrajectoryGeneration();
        } else {
            printError("Unknown TrajectoryGenerator type'" + type + "'.");
            return nullptr;
        }
    }

    TrajectoryEvaluator *ModuleFactory::parseTrajectoryEvaluators(std::string type) {
        if (type == "Naive") {
            return new trajectory_evaluators::Naive();
        } else if (type == "Frontier") {
            return new trajectory_evaluators::Frontier();
        } else {
            printError("Unknown TrajectoryEvaluator type'" + type + "'.");
            return nullptr;
        }
    }

    SegmentSelector *ModuleFactory::parseSegmentSelectors(std::string type) {
        if (type == "Greedy") {
            return new segment_selectors::Greedy();
        } else if (type == "RandomWeighted") {
            return new segment_selectors::RandomWeighted();
        } else {
            printError("Unknown SegmentSelector type '" + type + "'.");
            return nullptr;
        }
    }

    GeneratorUpdater *ModuleFactory::parseGeneratorUpdaters(std::string type) {
        if (type == "ResetTree") {
            return new generator_updaters::ResetTree();
        } else if (type == "UpdateNothing") {
            return new generator_updaters::UpdateNothing();
        } else if (type == "RecheckCollision") {
            return new generator_updaters::RecheckCollision();
        } else {
            printError("Unknown GeneratorUpdater type '" + type + "'.");
            return nullptr;
        }
    }

    CostComputer *ModuleFactory::parseCostComputers(std::string type) {
        if (type == "SegmentTime") {
            return new cost_computers::SegmentTime();
        } else if (type == "SegmentLength") {
            return new cost_computers::SegmentLength();
        } else {
            printError("Unknown CostComputer type '" + type + "'.");
            return nullptr;
        }
    }

    ValueComputer *ModuleFactory::parseValueComputers(std::string type) {
        if (type == "LinearValue") {
            return new value_computers::LinearValue();
        } else {
            ModuleFactory::Instance()->printError("Unknown ValueComputer type '" + type + "'.");
            return nullptr;
        }
    }

    NextSelector *ModuleFactory::parseNextSelectors(std::string type) {
        if (type == "ImmediateBest") {
            return new next_selectors::ImmediateBest();
        } else if (type == "SubsequentBest") {
            return new next_selectors::SubsequentBest();
        } else {
            printError("Unknown NextSelector type '" + type + "'.");
            return nullptr;
        }
    }

    EvaluatorUpdater *ModuleFactory::parseEvaluatorUpdaters(std::string type) {
        if (type == "UpdateNothing") {
            return new evaluator_updaters::UpdateNothing();
        }  else {
            printError("Unknown EvaluatorUpdater type '" + type + "'.");
            return nullptr;
        }
    }

    BackTracker *ModuleFactory::parseBackTrackers(std::string type) {
        if (type == "RotateInPlace") {
            return new back_trackers::RotateInPlace();
        } else if (type == "RotateReverse") {
            return new back_trackers::RotateReverse();
        } else {
            printError("Unknown BackTracker type '" + type + "'.");
            return nullptr;
        }
    }

    // Module factory core functionality
    ModuleFactory *ModuleFactory::instance_ = nullptr;

    ModuleFactory *ModuleFactory::Instance() {
        if (!instance_) {
            // Default to a ROS factory here...
            instance_ = new ModuleFactoryROS();
        }
        return instance_;
    }

    std::unique_ptr <TrajectoryGenerator> ModuleFactory::createTrajectoryGenerator(std::string args,
                                                                                   std::shared_ptr <voxblox::EsdfServer> voxblox_ptr,
                                                                                   bool verbose) {
        std::unique_ptr <TrajectoryGenerator> result = createCommon<TrajectoryGenerator>(std::string("Uniform"),
                                                                                         args,
                                                                                         &ModuleFactory::parseTrajectoryGenerators,
                                                                                         verbose);
        if (result) {
            result->setVoxbloxPtr(voxblox_ptr);
        }
        return result;
    }

    std::unique_ptr <TrajectoryEvaluator> ModuleFactory::createTrajectoryEvaluator(std::string args,
                                                                                   std::shared_ptr <voxblox::EsdfServer> voxblox_ptr,
                                                                                   bool verbose) {
        std::unique_ptr <TrajectoryEvaluator> result = createCommon<TrajectoryEvaluator>(std::string("Naive"),
                                                                                         args,
                                                                                         &ModuleFactory::parseTrajectoryEvaluators,
                                                                                         verbose);
        if (result) {
            result->setVoxbloxPtr(voxblox_ptr);
        }
        return result;
    }

    std::unique_ptr <SegmentSelector> ModuleFactory::createSegmentSelector(std::string args, bool verbose) {
        return createCommon<SegmentSelector>(std::string("Greedy"), args,
                                             &ModuleFactory::parseSegmentSelectors, verbose);
    }

    std::unique_ptr <GeneratorUpdater>
    ModuleFactory::createGeneratorUpdater(std::string args, TrajectoryGenerator *parent,
                                          bool verbose) {
        std::unique_ptr <GeneratorUpdater> result = createCommon<GeneratorUpdater>(std::string("ResetTree"),
                                                                                   args,
                                                                                   &ModuleFactory::parseGeneratorUpdaters,
                                                                                   verbose);
        if (result) {
            result->setParent(parent);
        }
        return result;
    }

    std::unique_ptr <CostComputer> ModuleFactory::createCostComputer(std::string args, bool verbose) {
        return createCommon<CostComputer>(std::string("SegmentTime"), args, &ModuleFactory::parseCostComputers, verbose);
    }

    std::unique_ptr <ValueComputer> ModuleFactory::createValueComputer(std::string args, bool verbose) {
        return createCommon<ValueComputer>(std::string("LinearValue"), args, &ModuleFactory::parseValueComputers,
                                           verbose);
    }

    std::unique_ptr <NextSelector> ModuleFactory::createNextSelector(std::string args, bool verbose) {
        return createCommon<NextSelector>(std::string("ImmediateBest"), args, &ModuleFactory::parseNextSelectors,
                                          verbose);
    }

    std::unique_ptr <EvaluatorUpdater>
    ModuleFactory::createEvaluatorUpdater(std::string args, TrajectoryEvaluator *parent,
                                          bool verbose) {
        std::unique_ptr <EvaluatorUpdater> result = createCommon<EvaluatorUpdater>(std::string("UpdateNothing"),
                                                                                   args,
                                                                                   &ModuleFactory::parseEvaluatorUpdaters,
                                                                                   verbose);
        if (result) {
            result->setParent(parent);
        }
        return result;
    }

    std::unique_ptr <BackTracker> ModuleFactory::createBackTracker(std::string args, bool verbose) {
        return createCommon<BackTracker>(std::string("RotateInPlace"), args, &ModuleFactory::parseBackTrackers,
                                         verbose);
    }

    // ROS factory
    bool ModuleFactoryROS::getParamMapAndType(Module::ParamMap *map, std::string *type, std::string args) {
        // For the ros factory, args is the namespace of the module
        ros::NodeHandle nh(args);
        std::vector <std::string> keys;
        std::string value;
        nh.getParamNames(keys);
        for (int i = 0; i > keys.size(); ++i) {
            nh.getParam(keys[i], value);
            (*map)[keys[i]] = value;
        }
        std::string type_default("");
        if (!nh.getParam("type", *type)) {
            type_default = " (default)";
        }
        (*map)["verbose_text"] = "Creating Module '" + *type + type_default + "' from namespace '"
                                 + args + "' with parameters:";
        return true;
    }

    void ModuleFactoryROS::printVerbose(const Module::ParamMap &map) {
        ROS_INFO("%s", map.at("verbose_text").c_str()); // Will warn about formatting otherwise
    }

    void ModuleFactoryROS::printError(std::string message) {
        ROS_ERROR("%s", message.c_str());   // Will warn about formatting otherwise
    }

} // namepsace mav_active_3d_planning