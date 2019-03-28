#include "mav_active_3d_planning/module_factory.h"

// ROS factory
#include <ros/param.h>
#include <ros/node_handle.h>
#include <ros/console.h>
#include <XmlRpcValue.h>

// Default types
#include "mav_active_3d_planning/trajectory_generator.h"
#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/defaults.h"


namespace mav_active_3d_planning {

    // Module factory core functionality
    ModuleFactory *ModuleFactory::instance_ = nullptr;

    ModuleFactory *ModuleFactory::Instance() {
        if (!instance_) {
            // Default to a ROS factory here...
            instance_ = new ModuleFactoryROS();
        }
        return instance_;
    }

    ModuleFactory::ModuleMap *ModuleFactory::module_map_ = nullptr;

    ModuleFactory::ModuleMap *ModuleFactory::getModuleMap() {
        if (!module_map_) {
            module_map_ = new ModuleMap();
        }
        return module_map_;
    }

    void ModuleFactory::getDefaultType(const std::type_index &module_index, std::string *type) {
        if (module_index == std::type_index(typeid(defaults::BoundingVolume))) {
            *type = "BoundingVolume";
        } else if (module_index == std::type_index(typeid(defaults::SystemConstraints))) {
            *type = "SystemConstraints";
        } else if (module_index == std::type_index(typeid(EvaluatorUpdater))) {
            *type = "EvaluatorUpdateNothing";
        } else if (module_index == std::type_index(typeid(GeneratorUpdater))) {
            *type = "GeneratorUpdateNothing";
        }
    }

    bool ModuleFactory::moduleParametrizationStep(Module::ParamMap *map, Module *module, bool verbose) {
        module->setupFromParamMap(map);
        module->assureParamsValid();
        if (verbose) { printVerbose(*map); }
        return true;
    }

    // ROS factory
    bool ModuleFactoryROS::getParamMapAndType(Module::ParamMap *map, std::string *type, std::string args) {
        // For the ros factory, args is the namespace of the module
        std::vector <std::string> keys;
        std::string key;
        XmlRpc::XmlRpcValue value;
        ros::NodeHandle nh(args);
        nh.getParamNames(keys);

        // parse all params
        for (int i = 0; i < keys.size(); ++i) {
            key = keys[i];
            if (key.find(args) != 0) {
                continue;
            }
            key = key.substr(args.length() + 1);
            if (key.find(std::string("/")) != std::string::npos) {
                continue;
            }
            nh.getParam(key, value);
            std::stringstream ss("");
            ss << value;
            (*map)[key] = ss.str();
        }

        // get type + verbose
        std::string type_default("");
        if (!nh.getParam("type", *type)) {
            type_default = " (default)";
        }
        (*map)["verbose_text"] = "Creating Module '" + *type + "'" + type_default + " from namespace '" + args +
                                 "' with parameters:";
        (*map)["param_namespace"] = args;   // also log the namespace for other modules as default
        return true;
    }

    void ModuleFactoryROS::printVerbose(const Module::ParamMap &map) {
        ROS_INFO("%s", map.at("verbose_text").c_str()); // Will warn about formatting otherwise
    }

    void ModuleFactoryROS::printError(const std::string &message) {
        ROS_ERROR("%s", message.c_str());   // Will warn about formatting otherwise
    }

} // namepsace mav_active_3d_planning