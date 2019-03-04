#ifndef MAV_ACTIVE_3D_PLANNING_MODULE_FACTORY_H
#define MAV_ACTIVE_3D_PLANNING_MODULE_FACTORY_H

#include "mav_active_3d_planning/module.h"
#include "mav_active_3d_planning/trajectory_generator.h"
#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/back_tracker.h"

#include <voxblox_ros/esdf_server.h>

#include <string>
#include <memory>
#include <map>

namespace mav_active_3d_planning {

    // Base class to create modules. Setup a concrete factory for different parametrization methods. Types of modules
    // are declared here, add new modules to the base class body.
    class ModuleFactory {
    public:
        // Singleton accessor
        static ModuleFactory* Instance();

        // Trajectory Generators
        static TrajectoryGenerator* createTrajectoryGenerator(voxblox::EsdfServer *voxblox_ptr, std::string param_ns);

        // Trajectory Evaluators
        static TrajectoryEvaluator* createTrajectoryEvaluator(voxblox::EsdfServer *voxblox_ptr, std::string param_ns);

        // TrajectoryGenerator -> selectSegment wrappers
        static SegmentSelector *createSegmentSelector(std::string param_ns);

        // TrajectoryGenerator -> updateSegments wrappers
        static GeneratorUpdater *createGeneratorUpdater(std::string param_ns, TrajectoryGenerator* parent);

        // TrajectoryEvaluator -> computeCost wrappers
        static CostComputer *createCostComputer(std::string param_ns);

        // TrajectoryEvaluator -> computeValue wrappers
        std::unique_ptr<ValueComputer> createValueComputer(std::string args, bool verbose);

        // TrajectoryEvaluator -> selectNextBest wrappers
        static NextSelector *createNextSelector(std::string param_ns);

        // TrajectoryEvaluator -> updateSegments wrappers
        static EvaluatorUpdater *createEvaluatorUpdater(std::string param_ns, TrajectoryEvaluator* parent);

        // BackTrackers
        static BackTracker *createBackTracker(std::string param_ns);

    protected:
        ModuleFactory() {}
        virtual ~ModuleFactory() {}

        // Singleton instance
        static ModuleFactory* instance_;

        // Create a param map from the arg string (ROS, string, file, ...)
        virtual bool getParamMapAndType(Module::ParamMap *map, std::string* type, std::string args) = 0;

        // Modules add verbose param text to the map. Implement this if you want to print the result.
        virtual void printVerbose(const Module::ParamMap &map) {}

        // Implement this to print type errors
        virtual void printError(const std::string &message) {}
    };

    // Concrete factory for ros param server
    class ModuleFactoryROS : public ModuleFactory {
        friend ModuleFactory;

    protected:
        ModuleFactoryROS() {}
        ~ModuleFactoryROS() {}

        // Implement virtual methods
        bool getParamMapAndType(Module::ParamMap *map, std::string* type, std::string args);
        void printVerbose(const Module::ParamMap &map);
        void printError(std::string message);
    };

}; // namepsace mav_active_3d_planning
#endif //MAV_ACTIVE_3D_PLANNING_MODULE_FACTORY_H
