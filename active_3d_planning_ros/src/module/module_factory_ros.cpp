#include <active_3d_planning_ros/module/module_factory_ros.h>

// ROS factory
#include <map>
#include <string>
#include <vector>

#include <XmlRpcValue.h>
#include <ros/console.h>
#include <ros/node_handle.h>

namespace active_3d_planning {
namespace ros {
// ROS factory
ModuleFactoryROS::ModuleFactoryROS() : ModuleFactory() {}

bool ModuleFactoryROS::getParamMapAndType(Module::ParamMap* map,
                                          std::string* type, std::string args) {
  // For the ros factory, args is the namespace of the module
  std::vector<std::string> keys;
  std::string key;
  XmlRpc::XmlRpcValue value;
  ::ros::NodeHandle nh(args);
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
  (*map)["verbose_text"] =
      "\n******************** New Module "
      "********************\nCreating Module '" +
      *type + "'" + type_default + " from namespace '" + args +
      "' with parameters:";
  (*map)["param_namespace"] =
      args;  // also log the args for other modules, this is key required for
             // all factories due to default args!
  return true;
}

void ModuleFactoryROS::printVerbose(const Module::ParamMap& map) {
  ROS_INFO(
      "%s",
      map.at("verbose_text").c_str());  // Will warn about formatting otherwise
}

void ModuleFactoryROS::printError(const std::string& message) {
  ROS_ERROR("%s", message.c_str());  // Will warn about formatting otherwise
}

}  // namespace ros
}  // namespace active_3d_planning
