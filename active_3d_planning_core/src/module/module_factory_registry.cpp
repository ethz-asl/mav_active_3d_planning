#include <active_3d_planning_core/module/module_factory_registry.h>

namespace active_3d_planning {

ModuleFactoryRegistry& ModuleFactoryRegistry::Instance() {
  static ModuleFactoryRegistry instance;
  return instance;
}

ModuleFactoryRegistry::ModuleFactoryRegistry() = default;

// ModuleFactoryRegistry::ModuleMap ModuleFactoryRegistry::module_map_;
}  // namespace active_3d_planning
