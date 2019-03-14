#include "mav_active_3d_planning/module_factory.h"
#include "mav_active_3d_planning/defaults.h"

#include <ros/param.h>
#include <ros/node_handle.h>
#include <ros/console.h>
#include <XmlRpcValue.h>

// Default modules
#include "mav_active_3d_planning/modules/trajectory_generators/segment_selectors/default_segment_selectors.h"
#include "mav_active_3d_planning/modules/trajectory_generators/generator_updaters/default_generator_updaters.h"
#include "mav_active_3d_planning/modules/trajectory_evaluators/cost_computers/default_cost_computers.h"
#include "mav_active_3d_planning/modules/trajectory_evaluators/value_computers/default_value_computers.h"
#include "mav_active_3d_planning/modules/trajectory_evaluators/next_selectors/default_next_selectors.h"
#include "mav_active_3d_planning/modules/trajectory_evaluators/evaluator_updaters/default_evaluator_updaters.h"
#include "mav_active_3d_planning/modules/back_trackers/default_back_trackers.h"

// Include available modules. Modules are to be included and created ONLY though the factory
#include "mav_active_3d_planning/modules/trajectory_generators/uniform.h"
#include "mav_active_3d_planning/modules/trajectory_generators/random_linear.h"
#include "mav_active_3d_planning/modules/trajectory_generators/rrt.h"
#include "mav_active_3d_planning/modules/trajectory_generators/mav_trajectory_generation.h"
#include "mav_active_3d_planning/modules/trajectory_evaluators/simulated_sensor_evaluators.h"
#include "mav_active_3d_planning/modules/trajectory_evaluators/sensor_models/camera_models.h"
#include "mav_active_3d_planning/modules/trajectory_evaluators/evaluator_updaters/simulated_sensor_updaters.h"



namespace mav_active_3d_planning {

    // Module lists
    TrajectoryGenerator *ModuleFactory::parseTrajectoryGenerators(std::string type) {
        if (type == "Uniform") {
            return new trajectory_generators::Uniform();
        } else if (type == "RandomLinear") {
            return new trajectory_generators::RandomLinear();
        } else if (type == "MavTrajectoryGeneration") {
            return new trajectory_generators::MavTrajectoryGeneration();
        } else if (type == "RRT") {
            return new trajectory_generators::RRT();
        } else {
            printError("Unknown TrajectoryGenerator type '" + type + "'.");
            return nullptr;
        }
    }

    TrajectoryEvaluator *ModuleFactory::parseTrajectoryEvaluators(std::string type) {
        if (type == "NaiveEvaluator") {
            return new trajectory_evaluators::NaiveEvaluator();
        } else if (type == "Frontier") {
            return new trajectory_evaluators::Frontier();
        } else if (type == "VoxelType") {
            return new trajectory_evaluators::VoxelType();
        } else {
            printError("Unknown TrajectoryEvaluator type '" + type + "'.");
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
        } else if (type == "ExponentialDiscount") {
            return new value_computers::ExponentialDiscount();
        } else if (type == "Accumulate") {
            return new value_computers::Accumulate();
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
        } else if (type == "ResetTree") {
            return new evaluator_updaters::ResetTree();
        } else if (type == "UpdateAll") {
            return new evaluator_updaters::UpdateAll();
        } else if (type == "PruneByValue") {
            return new evaluator_updaters::PruneByValue();
        } else if (type == "Periodic") {
            return new evaluator_updaters::Periodic();
        } if (type == "SimulatedSensorUpdater") {
            return new evaluator_updaters::SimulatedSensorUpdater();
        } else {
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

    SensorModel *ModuleFactory::parseSensorModels(std::string type) {
        if (type == "SimpleRayCaster") {
            return new sensor_models::SimpleRayCaster();
        } else if (type == "IterativeRayCaster") {
            return new sensor_models::IterativeRayCaster();
        } else {
            printError("Unknown SensorModel type '" + type + "'.");
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

    void ModuleFactory::parametrizeModule(std::string args, Module *module) {
        Module::ParamMap map;
        std::string type;
        getParamMapAndType(&map, &type, args);
        module->setupFromParamMap(&map);
    }

    bool ModuleFactory::setupCommon(Module::ParamMap *map, Module *module, bool verbose) {
        module->setupFromParamMap(map);
        module->assureParamsValid();
        if (verbose) { printVerbose(*map); }
        return true;
    }

    std::unique_ptr <TrajectoryGenerator> ModuleFactory::createTrajectoryGenerator(std::string args,
                                                                                   std::shared_ptr <voxblox::EsdfServer> voxblox_ptr,
                                                                                   bool verbose) {
        std::unique_ptr <TrajectoryGenerator> result;
        Module::ParamMap map;
        if (!createCommon<TrajectoryGenerator>(&map, &result, std::string("Uniform"), args,
                                               &ModuleFactory::parseTrajectoryGenerators, verbose)) {
            return nullptr;
        }
        result->setVoxbloxPtr(voxblox_ptr);
        setupCommon(&map, result.get(), verbose);
        return result;
    }

    std::unique_ptr <TrajectoryEvaluator> ModuleFactory::createTrajectoryEvaluator(std::string args,
                                                                                   std::shared_ptr <voxblox::EsdfServer> voxblox_ptr,
                                                                                   bool verbose) {
        std::unique_ptr <TrajectoryEvaluator> result;
        Module::ParamMap map;
        if (!createCommon<TrajectoryEvaluator>(&map, &result, std::string("NaiveEvaluator"), args,
                                               &ModuleFactory::parseTrajectoryEvaluators, verbose)) {
            return nullptr;
        }
        result->setVoxbloxPtr(voxblox_ptr);
        setupCommon(&map, result.get(), verbose);
        return result;
    }

    std::unique_ptr <SegmentSelector> ModuleFactory::createSegmentSelector(std::string args, bool verbose) {
        std::unique_ptr <SegmentSelector> result;
        Module::ParamMap map;
        if (!createCommon<SegmentSelector>(&map, &result, std::string("Greedy"), args,
                                           &ModuleFactory::parseSegmentSelectors, verbose)) {
            return nullptr;
        }
        setupCommon(&map, result.get(), verbose);
        return result;
    }

    std::unique_ptr <GeneratorUpdater>
    ModuleFactory::createGeneratorUpdater(std::string args, TrajectoryGenerator *parent, bool verbose) {
        std::unique_ptr <GeneratorUpdater> result;
        Module::ParamMap map;
        if (!createCommon<GeneratorUpdater>(&map, &result, std::string("ResetTree"), args,
                                            &ModuleFactory::parseGeneratorUpdaters, verbose)) {
            return nullptr;
        }
        result->setParent(parent);
        setupCommon(&map, result.get(), verbose);
        return result;
    }

    std::unique_ptr <CostComputer> ModuleFactory::createCostComputer(std::string args, bool verbose) {
        std::unique_ptr <CostComputer> result;
        Module::ParamMap map;
        if (!createCommon<CostComputer>(&map, &result, std::string("SegmentTime"), args,
                                        &ModuleFactory::parseCostComputers, verbose)) {
            return nullptr;
        }
        setupCommon(&map, result.get(), verbose);
        return result;
    }

    std::unique_ptr <ValueComputer> ModuleFactory::createValueComputer(std::string args, bool verbose) {
        std::unique_ptr <ValueComputer> result;
        Module::ParamMap map;
        if (!createCommon<ValueComputer>(&map, &result, std::string("LinearValue"), args,
                                         &ModuleFactory::parseValueComputers, verbose)) {
            return nullptr;
        }
        setupCommon(&map, result.get(), verbose);
        return result;
    }

    std::unique_ptr <NextSelector> ModuleFactory::createNextSelector(std::string args, bool verbose) {
        std::unique_ptr <NextSelector> result;
        Module::ParamMap map;
        if (!createCommon<NextSelector>(&map, &result, std::string("ImmediateBest"), args,
                                        &ModuleFactory::parseNextSelectors, verbose)) {
            return nullptr;
        }
        setupCommon(&map, result.get(), verbose);
        return result;
    }

    std::unique_ptr <EvaluatorUpdater>
    ModuleFactory::createEvaluatorUpdater(std::string args, TrajectoryEvaluator *parent, bool verbose) {
        std::unique_ptr <EvaluatorUpdater> result;
        Module::ParamMap map;
        if (!createCommon<EvaluatorUpdater>(&map, &result, std::string("UpdateNothing"), args,
                                            &ModuleFactory::parseEvaluatorUpdaters, verbose)) {
            return nullptr;
        }
        result->setParent(parent);
        setupCommon(&map, result.get(), verbose);
        return result;
    }

    std::unique_ptr <BackTracker> ModuleFactory::createBackTracker(std::string args, bool verbose) {
        std::unique_ptr <BackTracker> result;
        Module::ParamMap map;
        if (!createCommon<BackTracker>(&map, &result, std::string("RotateInPlace"), args,
                                       &ModuleFactory::parseBackTrackers, verbose)) {
            return nullptr;
        }
        setupCommon(&map, result.get(), verbose);
        return result;
    }

    std::unique_ptr <SensorModel> ModuleFactory::createSensorModel(std::string args,
                                                                   std::shared_ptr <voxblox::EsdfServer> voxblox_ptr,
                                                                   bool verbose) {
        std::unique_ptr <SensorModel> result;
        Module::ParamMap map;
        if (!createCommon<SensorModel>(&map, &result, std::string("SimpleRayCaster"), args,
                                       &ModuleFactory::parseSensorModels, verbose)) {
            return nullptr;
        }
        result->setVoxbloxPtr(voxblox_ptr);
        setupCommon(&map, result.get(), verbose);
        return result;
    }

    // ROS factory
    bool ModuleFactoryROS::getParamMapAndType(Module::ParamMap *map, std::string *type, std::string args) {
        // For the ros factory, args is the namespace of the module
        std::vector <std::string> keys;
        std::string key;
        XmlRpc::XmlRpcValue value;
        ros::NodeHandle nh(args);
        nh.getParamNames(keys);

        // parse all params
        for (int i = 0; i < keys.size(); ++i) {
            key = keys[i];
            if (key.find(args) != 0) {
                continue;
            }
            key = key.substr(args.length() + 1);
            if (key.find(std::string("/")) != std::string::npos) {
                continue;
            }
            nh.getParam(key, value);
            std::stringstream ss("");
            ss << value;
            (*map)[key] = ss.str();
        }

        // get type + verbose
        std::string type_default("");
        if (!nh.getParam("type", *type)) {
            type_default = " (default)";
        }
        (*map)["verbose_text"] = "Creating Module '" + *type + "'" + type_default + " from namespace '" + args +
                                 "' with parameters:";
        (*map)["param_namespace"] = args;   // also log the namespace for other modules as default
        return true;
    }

    void ModuleFactoryROS::printVerbose(const Module::ParamMap &map) {
        ROS_INFO("%s", map.at("verbose_text").c_str()); // Will warn about formatting otherwise
    }

    void ModuleFactoryROS::printError(const std::string &message) {
        ROS_ERROR("%s", message.c_str());   // Will warn about formatting otherwise
    }

} // namepsace mav_active_3d_planning