#include <active_3d_planning_core/module/module_factory.h>

// Default types
#include <map>
#include <string>

#include "active_3d_planning_core/data/bounding_volume.h"
#include "active_3d_planning_core/data/system_constraints.h"
#include "active_3d_planning_core/module/trajectory_evaluator.h"
#include "active_3d_planning_core/module/trajectory_generator.h"

namespace active_3d_planning {

void ModuleFactory::getDefaultType(const std::type_index& module_index,
                                   std::string* type) {
  if (module_index == std::type_index(typeid(BoundingVolume))) {
    *type = "BoundingVolume";
  } else if (module_index == std::type_index(typeid(SystemConstraints))) {
    *type = "SystemConstraints";
  } else if (module_index == std::type_index(typeid(EvaluatorUpdater))) {
    *type = "UpdateNothing";
  } else if (module_index == std::type_index(typeid(GeneratorUpdater))) {
    *type = "UpdateNothingGenerator";
  }
}

void ModuleFactory::registerLinkableModule(const std::string& name,
                                           Module* module) {
  linkable_module_list_[name] = module;
}

Module* ModuleFactory::readLinkableModule(const std::string& name) {
  std::map<std::string, Module*>::iterator it =
      linkable_module_list_.find(name);
  if (it == linkable_module_list_.end()) {
    printError("No module with name '" + name +
               "' is registered in the linkable modules list.");
    return nullptr;
  }
  return it->second;
}

}  // namespace active_3d_planning
