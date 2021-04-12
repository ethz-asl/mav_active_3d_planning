#include <string>

#include <active_3d_planning_core/module/module.h>

namespace active_3d_planning {

ModuleBase::ModuleBase() : verbose_modules_(false) {}

// Throw an exception if module parametrization is invalid. Call this after
// module constructors.
void ModuleBase::assureParamsValid() {
  std::string error_message("");  // default error
  if (!checkParamsValid(&error_message)) {
    throw std::invalid_argument("Invalid module parameters: " + error_message);
  }
}

void ModuleBase::setVerbose(bool verbose) { verbose_modules_ = verbose; }

Module::Module(PlannerI& planner) : planner_(planner) {}

}  // namespace active_3d_planning
