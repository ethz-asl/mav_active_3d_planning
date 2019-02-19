#include "mav_active_3d_planning/trajectory_generator.h"
#include "mav_active_3d_planning/module_factory.h"

#include <ros/param.h>

namespace mav_active_3d_planning {

    TrajectoryGenerator::TrajectoryGenerator(voxblox::EsdfServer *voxblox_ptr, std::string param_ns)
            : voxblox_ptr_(voxblox_ptr),
              bounding_volume_(param_ns + "/bounding_volume"),
              segment_selector_(nullptr),
              generator_updater_(nullptr),
              p_namespace_(param_ns) {
        // Parameters
        ros::param::param<bool>(param_ns + "/collision_optimistic", p_collision_optimistic_, false);
        ros::param::param<double>(param_ns + "/collision_radius", p_collision_radius_, 0.35);

        //Setup the voxblox server collision for visualization
        voxblox_ptr_->setTraversabilityRadius(static_cast<float>(p_collision_radius_));
    }

    bool TrajectoryGenerator::checkTraversable(const Eigen::Vector3d &position) {
        if (!bounding_volume_.contains(position)) {
            return false;
        }
        if (!voxblox_ptr_->getEsdfMapPtr()->isObserved(position)) {
            return p_collision_optimistic_;
        }
        double distance = 0.0;
        voxblox_ptr_->getEsdfMapPtr()->getDistanceAtPosition(position, &distance);
        return (distance > p_collision_radius_);
    }

    TrajectorySegment* TrajectoryGenerator::selectSegment(TrajectorySegment &root){
        // If not implemented use a (default) module
        if (segment_selector_ == nullptr){
            segment_selector_ = ModuleFactory::createSegmentSelector(p_namespace_+"/segment_selector");
        }
        return segment_selector_->selectSegment(root);
    }

    bool TrajectoryGenerator::updateSegments(TrajectorySegment &root){
        // If not implemented use a (default) module
        if (generator_updater_ == nullptr){
            generator_updater_ = ModuleFactory::createGeneratorUpdater(p_namespace_+"/generator_updater", this);
        }
        return generator_updater_->updateSegments(root);
    }

}  // namespace mav_active_3d_planning
