#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATOR_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATOR_H

#include "mav_active_3d_planning/trajectory_segment.h"
#include "mav_active_3d_planning/defaults.h"

#include <voxblox_ros/esdf_server.h>
#include <Eigen/Core>

#include <string>

namespace mav_active_3d_planning {

    // Forward declaration
    class CostComputer;
    class ValueComputer;
    class NextSelector;
    class EvaluatorUpdater;

    // Base class for trajectory evaluators to provide uniform interface with other classes
    class TrajectoryEvaluator {
    public:
        TrajectoryEvaluator(voxblox::EsdfServer *voxblox_ptr, std::string param_ns);

        virtual ~TrajectoryEvaluator() {}

        // compute the gain of a TrajectorySegment
        virtual bool computeGain(TrajectorySegment &traj_in) = 0;

        // compute the cost of a TrajectorySegment
        virtual bool computeCost(TrajectorySegment &traj_in);

        // compute the Value of a segment with known cost and gain
        virtual bool computeValue(TrajectorySegment &traj_in);

        // return the index of the most promising child segment
        virtual int selectNextBest(TrajectorySegment &traj_in);

        // Whether and how to update existing segments when a new trajectory is executed
        virtual bool updateSegments(TrajectorySegment &root);

    protected:
        // Voxblox map
        voxblox::EsdfServer *voxblox_ptr_;

        // bounding volume of interesting target
        defaults::BoundingVolume bounding_volume_;

        // params
        std::string p_namespace_;

        // default modules
        CostComputer* cost_computer_;
        ValueComputer* value_computer_;
        NextSelector* next_selector_;
        EvaluatorUpdater* evaluator_updater_;
    };

    // Abstract encapsulation for default/modular implementations of the computeCost method
    class CostComputer {
    public:
        virtual bool computeCost(TrajectorySegment &traj_in) = 0;
    };

    // Abstract encapsulation for default/modular implementations of the computeValue method
    class ValueComputer {
    public:
        virtual bool computeValue(TrajectorySegment &traj_in) = 0;
    };

    // Abstract encapsulation for default/modular implementations of the selectNextBest method
    class NextSelector {
    public:
        virtual int selectNextBest(TrajectorySegment &traj_in) = 0;
    };

    // Abstract encapsulation for default/modular implementations of the updateSegments method
    class EvaluatorUpdater {
    public:
        EvaluatorUpdater(TrajectoryEvaluator* parent = nullptr) : parent_(parent) {};

        virtual bool updateSegments(TrajectorySegment &root) = 0;

    protected:
        TrajectoryEvaluator* parent_;
    };

}  // namespace mav_active_3d_planning
#endif //MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATOR_H
