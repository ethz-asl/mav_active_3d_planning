#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/module_factory.h"

#include <ros/param.h>

#include <vector>

namespace mav_active_3d_planning {

    TrajectoryEvaluator::TrajectoryEvaluator(voxblox::EsdfServer *voxblox_ptr, std::string param_ns)
            : voxblox_ptr_(voxblox_ptr),
              bounding_volume_(param_ns + "/bounding_volume"),
              cost_computer_(nullptr),
              value_computer_(nullptr),
              next_selector_(nullptr),
              evaluator_updater_(nullptr),
              p_namespace_(param_ns) {}

    bool TrajectoryEvaluator::computeCost(TrajectorySegment &traj_in) {
        // If not implemented use a (default) module
        if (cost_computer_ == nullptr){
            cost_computer_ = ModuleFactory::createCostComputer(p_namespace_+"/cost_computer");
        }
        return cost_computer_->computeCost(traj_in);
    }

    bool TrajectoryEvaluator::computeValue(TrajectorySegment &traj_in) {
        // If not implemented use a (default) module
        if (value_computer_ == nullptr){
            value_computer_ = ModuleFactory::createValueComputer(p_namespace_+"/value_computer");
        }
        return value_computer_->computeValue(traj_in);
    }

    int TrajectoryEvaluator::selectNextBest(TrajectorySegment &traj_in) {
        // If not implemented use a (default) module
        if (next_selector_ == nullptr){
            next_selector_ = ModuleFactory::createNextSelector(p_namespace_+"/next_selector");
        }
        return next_selector_->selectNextBest(traj_in);
    }

    bool TrajectoryEvaluator::updateSegments(TrajectorySegment &root) {
        // If not implemented use a (default) module
        if (evaluator_updater_ == nullptr){
            evaluator_updater_ = ModuleFactory::createEvaluatorUpdater(p_namespace_+"/evaluator_updater", this);
        }
        return evaluator_updater_->updateSegments(root);
    }

} // namepsace mav_active_3d_planning