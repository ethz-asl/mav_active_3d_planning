#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_MODULE_FACTORY_REGISTRY_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_MODULE_FACTORY_REGISTRY_H_

#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <utility>

#include <glog/logging.h>

#include "active_3d_planning_core/module/module.h"

namespace active_3d_planning {

class PlannerI;
class Module;
using ModuleFactoryMethod =
    std::function<Module*(PlannerI& planner)>;  // NOLINT

// Static Registry for Modules to register
class ModuleFactoryRegistry {
 public:
  virtual ~ModuleFactoryRegistry() = default;

  static ModuleFactoryRegistry& Instance();

  // Module creation (this is the main accessor for new modules).
  template <class ModuleType>
  std::unique_ptr<ModuleType> createModule(const std::string& module_name,
                                           PlannerI& planner,  // NOLINT
                                           bool verbose) {
    // Build requested module
    if (getRegistryMap().find(module_name) == getRegistryMap().end()) {
      const std::string typeList =
          getRegistryMap().empty()
              ? std::string()
              : std::accumulate(
                    std::next(getRegistryMap().begin()), getRegistryMap().end(),
                    getRegistryMap().begin()->first,
                    [](std::string& a,
                       std::pair<std::string, ModuleFactoryMethod> b) {
                      return a + ", " + b.first;
                    });

      LOG(FATAL) << "Could not find type '" << module_name
                 << "' to create a module from. Available types are: "
                 << typeList << ".";
      return nullptr;
    }
    return std::unique_ptr<ModuleType>(
        dynamic_cast<ModuleType*>(getRegistryMap().at(module_name)(planner)));
  }

  // Allow modules to register themselves to the factory using a static
  // Registration struct
  template <class ModuleType>
  struct Registration {
    explicit Registration(const std::string& module_name) {
      if (getRegistryMap().find(module_name) == getRegistryMap().end()) {
        getRegistryMap().insert(std::pair<std::string, ModuleFactoryMethod>(
            module_name,
            [](PlannerI& planner) { return new ModuleType(planner); }));
      } else {
        LOG(FATAL)
            << "ModuleFactory: Cannot register already existing module name '"
            << module_name << "'!";
      }
    }
  };

 private:
  ModuleFactoryRegistry();

  // this should be an unordered map for runtime I think
  typedef std::map<std::string, ModuleFactoryMethod> ModuleMap;

  static ModuleMap& getRegistryMap() {
    static ModuleMap module_map_;
    return module_map_;
  }
};

}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_MODULE_FACTORY_REGISTRY_H_
