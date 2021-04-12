#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_MODULE_FACTORY_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_MODULE_FACTORY_H_

#include <map>
#include <memory>
#include <string>
#include <typeindex>
#include <typeinfo>

#include "active_3d_planning_core/module/module.h"
#include "active_3d_planning_core/module/module_factory_registry.h"

namespace active_3d_planning {

// Base class to create modules. Setup a concrete factory for different
// parametrization methods.
class ModuleFactory {
 public:
  ModuleFactory() = default;

  virtual ~ModuleFactory() = default;

  // Module creation (this is the main accessor for new modules).
  template <class ModuleType>
  std::unique_ptr<ModuleType> createModule(const std::string& args,
                                           PlannerI& planner,  // NOLINT
                                           bool verbose) {
    // Get config
    Module::ParamMap param_map;
    std::string type = "NoDefaultTypeAvailable";
    getDefaultType(std::type_index(typeid(ModuleType)), &type);
    getParamMapAndType(&param_map, &type, args);
    auto module = ModuleFactoryRegistry::Instance().createModule<ModuleType>(
        type, planner, verbose);

    // Setup module
    module->setVerbose(verbose);
    module->setupFromParamMap(&param_map);
    module->assureParamsValid();
    if (verbose) {
      printVerbose(param_map);
    }
    return module;
  }

  // Modules that need to be linkable (eg. an evaluator-updater pair) can
  // register here
  void registerLinkableModule(const std::string& name, Module* module);

  // Modules that need to be linkable (eg. an evaluator-updater pair) can be
  // retrieved here
  Module* readLinkableModule(const std::string& name);

  // Derived Factory responsibilities
  // Create a param map from the arg string (ROS, string, file, ...)
  virtual bool getParamMapAndType(Module::ParamMap* map, std::string* type,
                                  std::string args) = 0;

 protected:
  // Specify default types for modules that need such (e.g. doNothing modules)
  void getDefaultType(const std::type_index& module_index, std::string* type);

  // Modules add verbose param text to the map. Implement this if you want to
  // print the result.
  virtual void printVerbose(const Module::ParamMap& map) = 0;

  // Implement this to print type errors
  virtual void printError(const std::string& message) = 0;

  // Modules that need to be linkable (eg. an evaluator-updater pair) can
  // register and read here
  std::map<std::string, Module*> linkable_module_list_;
};

}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_MODULE_FACTORY_H_
