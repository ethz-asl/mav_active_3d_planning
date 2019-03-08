#ifndef MAV_ACTIVE_3D_PLANNING_MODULE_FACTORY_H
#define MAV_ACTIVE_3D_PLANNING_MODULE_FACTORY_H

#include "mav_active_3d_planning/module.h"
#include "mav_active_3d_planning/trajectory_generator.h"
#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/back_tracker.h"

#include <voxblox_ros/esdf_server.h>

#include <string>
#include <memory>

namespace mav_active_3d_planning {

    // Base class to create modules. Setup a concrete factory for different parametrization methods. Types of modules
    // are declared here, add new modules to the base class body.
    class ModuleFactory {
    public:
        virtual ~ModuleFactory() {}

        // Singleton accessor
        static ModuleFactory *Instance();

        // Method to allow structs to parametrize themselves
        void parametrizeModule(std::string args, Module* module);

        // Module creation accessors
        // Trajectory Generators
        std::unique_ptr <TrajectoryGenerator> createTrajectoryGenerator(std::string args,
                                                                        std::shared_ptr <voxblox::EsdfServer> voxblox_ptr,
                                                                        bool verbose);

        // Trajectory Evaluators
        std::unique_ptr <TrajectoryEvaluator> createTrajectoryEvaluator(std::string args,
                                                                        std::shared_ptr <voxblox::EsdfServer> voxblox_ptr,
                                                                        bool verbose);

        // TrajectoryGenerator -> selectSegment wrappers
        std::unique_ptr <SegmentSelector> createSegmentSelector(std::string args, bool verbose);

        // TrajectoryGenerator -> updateSegments wrappers
        std::unique_ptr <GeneratorUpdater> createGeneratorUpdater(std::string args, TrajectoryGenerator *parent,
                                                                  bool verbose);

        // TrajectoryEvaluator -> computeCost wrappers
        std::unique_ptr <CostComputer> createCostComputer(std::string args, bool verbose);

        // TrajectoryEvaluator -> computeValue wrappers
        std::unique_ptr <ValueComputer> createValueComputer(std::string args, bool verbose);

        // TrajectoryEvaluator -> selectNextBest wrappers
        std::unique_ptr <NextSelector> createNextSelector(std::string args, bool verbose);

        // TrajectoryEvaluator -> updateSegments wrappers
        std::unique_ptr <EvaluatorUpdater> createEvaluatorUpdater(std::string args, TrajectoryEvaluator *parent,
                                                                  bool verbose);

        // BackTrackers
        std::unique_ptr <BackTracker> createBackTracker(std::string args, bool verbose);

    protected:
        ModuleFactory() {}

        // Singleton instance
        static ModuleFactory *instance_;

        // Create a param map from the arg string (ROS, string, file, ...)
        virtual bool getParamMapAndType(Module::ParamMap *map, std::string *type, std::string args) = 0;

        // Modules add verbose param text to the map. Implement this if you want to print the result.
        virtual void printVerbose(const Module::ParamMap &map) {}

        // Implement this to print type errors
        virtual void printError(const std::string &message) {}

        // Module parsers
        TrajectoryGenerator *parseTrajectoryGenerators(std::string type);

        TrajectoryEvaluator *parseTrajectoryEvaluators(std::string type);

        SegmentSelector *parseSegmentSelectors(std::string type);

        GeneratorUpdater *parseGeneratorUpdaters(std::string type);

        CostComputer *parseCostComputers(std::string type);

        ValueComputer *parseValueComputers(std::string type);

        NextSelector *parseNextSelectors(std::string type);

        EvaluatorUpdater *parseEvaluatorUpdaters(std::string type);

        BackTracker *parseBackTrackers(std::string type);

        // Base routine for creating modules
        template<class T>
        std::unique_ptr <T> createCommon(std::string default_type, std::string args,
                                         T *(ModuleFactory::*parse_function)(std::string), bool verbose) {
            Module::ParamMap map;
            std::string type = default_type;
            getParamMapAndType(&map, &type, args);
            T *result = (this->*parse_function)(type);
            if (!result) {
                // parsing failed
                return nullptr;
            }
            result->setupFromParamMap(&map);
            result->verbose_modules_ = verbose;
            result->assureParamsValid();
            if (verbose) { printVerbose(map); }
            return std::unique_ptr<T>(result);
        }
    };

    // Concrete factory for ros param server
    class ModuleFactoryROS : public ModuleFactory {
        friend ModuleFactory;

    public:
        ~ModuleFactoryROS() {}

    protected:
        ModuleFactoryROS() {}

        // Implement virtual methods
        bool getParamMapAndType(Module::ParamMap *map, std::string *type, std::string args);

        void printVerbose(const Module::ParamMap &map);

        void printError(std::string message);
    };

} // namepsace mav_active_3d_planning
#endif //MAV_ACTIVE_3D_PLANNING_MODULE_FACTORY_H
