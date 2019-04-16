#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATOR_H_
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATOR_H_

#include "mav_active_3d_planning/trajectory_segment.h"
#include "mav_active_3d_planning/module_factory.h"
#include "mav_active_3d_planning/defaults.h"

#include <voxblox_ros/esdf_server.h>
#include <Eigen/Core>

#include <string>
#include <vector>

namespace mav_active_3d_planning {

    // Forward declaration
    class SegmentSelector;
    class GeneratorUpdater;

    // Base class for trajectory generation to provide uniform interface with other classes
    class TrajectoryGenerator : public Module {
        friend ModuleFactory;

    public:
        virtual ~TrajectoryGenerator() {}

        // Expansion policy where to expand (from full tree)
        virtual bool selectSegment(TrajectorySegment **result, TrajectorySegment *root);

        // Expand a selected trajectory segment. Return true for successful expansion.
        virtual bool expandSegment(TrajectorySegment *target, std::vector<TrajectorySegment*> *new_segments) = 0;

        // Whether and how to update existing segments when a new trajectory is executed
        virtual bool updateSegments(TrajectorySegment *root);

        // Utility function for collision checking. Returns true if the position is reachable.
        bool checkTraversable(const Eigen::Vector3d &position);

    protected:
        TrajectoryGenerator() {}

        // Pointer to the esdf server for collision checking (and others)
        std::shared_ptr<voxblox::EsdfServer> voxblox_ptr_;

        // bounding box
        std::unique_ptr<defaults::BoundingVolume> bounding_volume_;
        std::unique_ptr<defaults::SystemConstraints> system_constraints_;

        // default modules
        std::unique_ptr<SegmentSelector> segment_selector_;
        std::unique_ptr<GeneratorUpdater> generator_updater_;

        // Parameters
        bool p_collision_optimistic_;
        double p_collision_radius_;
        std::string p_selector_args_;
        std::string p_updater_args_;

        // factory setup accessor
        virtual void setupFromParamMap(Module::ParamMap *param_map);
    };

    // Abstract encapsulation for default/modular implementations of the selectSegment method
    class SegmentSelector : public Module {
    public:
        virtual bool selectSegment(TrajectorySegment **result, TrajectorySegment *root) = 0;
    };

    // Abstract encapsulation for default/modular implementations of the updateSegments method
    class GeneratorUpdater : public Module {
    public:
        virtual bool updateSegments(TrajectorySegment *root) = 0;
    };

}  // namespace mav_active_3d_planning
#endif  // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATOR_H_
