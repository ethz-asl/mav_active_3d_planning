#include <active_3d_planning/module/module.h>

#include <active_3d_planning/planner/planner_I.h>

namespace active_3d_planning {

Module::Module(PlannerI& planner) : verbose_modules_(false), planner_(planner){}

// Throw an exception if module parametrization is invalid. Call this after module constructors.
void Module::assureParamsValid() {
    std::string error_message(""); // default error
    if (!checkParamsValid(&error_message)){
        throw std::invalid_argument("Invalid module parameters: " + error_message);
    }
}

void Module::setVerbose(bool verbose){
    verbose_modules_ = verbose;
}

} // namepsace active_3d_planning