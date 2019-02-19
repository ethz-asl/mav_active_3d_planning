#ifndef MAV_ACTIVE_3D_PLANNING_COMPONENT_FACTORY_H
#define MAV_ACTIVE_3D_PLANNING_COMPONENT_FACTORY_H

#include "mav_active_3d_planning/trajectory_generator.h"
#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/back_tracker.h"

#include <voxblox_ros/esdf_server.h>
#include <string>

namespace mav_active_3d_planning {

    // Static class to generate all encapsulated functionalities/modules. Types of functionalities are declared here,
    // add new modules in the body
    class ModuleFactory {
    public:
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
        static ValueComputer *createValueComputer(std::string param_ns);

        // TrajectoryEvaluator -> selectNextBest wrappers
        static NextSelector *createNextSelector(std::string param_ns);

        // TrajectoryEvaluator -> updateSegments wrappers
        static EvaluatorUpdater *createEvaluatorUpdater(std::string param_ns, TrajectoryEvaluator* parent);

        // BackTrackers
        static BackTracker *createBackTracker(std::string param_ns);

    private:
        ModuleFactory() {}
        virtual ~ModuleFactory() {}
    };

} // namepsace mav_active_3d_planning
#endif //MAV_ACTIVE_3D_PLANNING_COMPONENT_FACTORY_H
