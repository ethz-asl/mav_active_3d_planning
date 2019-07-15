#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATOR_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATOR_H

#include "mav_active_3d_planning/trajectory_segment.h"
#include "mav_active_3d_planning/module_factory.h"
#include "mav_active_3d_planning/defaults.h"

#include <voxblox_ros/esdf_server.h>
#include <Eigen/Core>
#include <visualization_msgs/Marker.h>

#include <string>
#include <memory>

namespace mav_active_3d_planning {

    // Forward declaration
    class CostComputer;
    class ValueComputer;
    class NextSelector;
    class EvaluatorUpdater;

    // Base class for trajectory evaluators to provide uniform interface with other classes
    class TrajectoryEvaluator : public Module {
    public:
        TrajectoryEvaluator(std::shared_ptr<voxblox::EsdfServer> voxblox_ptr, std::string param_ns);

        virtual ~TrajectoryEvaluator() {}

        // compute the gain of a TrajectorySegment
        virtual bool computeGain(TrajectorySegment *traj_in) = 0;

        // compute the cost of a TrajectorySegment
        virtual bool computeCost(TrajectorySegment *traj_in);

        // compute the Value of a segment with known cost and gain
        virtual bool computeValue(TrajectorySegment *traj_in);

        // return the index of the most promising child segment
        virtual int selectNextBest(TrajectorySegment *traj_in);

        // Whether and how to update existing segments when a new trajectory is executed
        virtual bool updateSegments(TrajectorySegment *root);

        // Implement this method to allow visualization of the information gain during simulation
        virtual void visualizeTrajectoryValue(visualization_msgs::MarkerArray* msg, const TrajectorySegment &trajectory) {}

    protected:
        friend ModuleFactory;

        TrajectoryEvaluator() {}

        // Voxblox map
        std::shared_ptr<voxblox::EsdfServer> voxblox_ptr_;

        // bounding volume of interesting target
        std::unique_ptr<defaults::BoundingVolume> bounding_volume_;

        // params
        std::string p_cost_args_;
        std::string p_value_args_;
        std::string p_next_args_;
        std::string p_updater_args_;

        // default modules
        std::unique_ptr<CostComputer> cost_computer_;
        std::unique_ptr<ValueComputer> value_computer_;
        std::unique_ptr<NextSelector> next_selector_;
        std::unique_ptr<EvaluatorUpdater> evaluator_updater_;

        // factory accessors
        virtual void setupFromParamMap(Module::ParamMap *param_map);
    };

    // Abstract encapsulation for default/modular implementations of the computeCost method
    class CostComputer : public Module {
    public:
        virtual bool computeCost(TrajectorySegment *traj_in) = 0;
    };

    // Abstract encapsulation for default/modular implementations of the computeValue method
    class ValueComputer : public Module {
    public:
        virtual bool computeValue(TrajectorySegment *traj_in) = 0;
    };

    // Abstract encapsulation for default/modular implementations of the selectNextBest method
    class NextSelector : public Module {
    public:
        virtual int selectNextBest(TrajectorySegment *traj_in) = 0;
    };

    // Abstract encapsulation for default/modular implementations of the updateSegments method
    class EvaluatorUpdater : public Module {
    public:
        virtual bool updateSegments(TrajectorySegment *root) = 0;
    };

}  // namespace mav_active_3d_planning
#endif //MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATOR_H
