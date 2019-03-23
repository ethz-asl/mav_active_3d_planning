#ifndef MAV_ACTIVE_3D_PLANNING_MODULE_FACTORY_H
#define MAV_ACTIVE_3D_PLANNING_MODULE_FACTORY_H

#include <voxblox_ros/esdf_server.h>

#include <string>
#include <memory>
#include <map>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <typeinfo>
#include <typeindex>

namespace mav_active_3d_planning {

    // Base class for all modules, provides creation functionalities through the factory
    class Module{
    public:
        typedef std::map<std::string, std::string> ParamMap;
        virtual ~Module() {}

    protected:
        friend class ModuleFactory;

        Module() : verbose_modules_(false) {}

        // factory method that sets up the module from a param map
        virtual void setupFromParamMap(ParamMap *param_map) = 0;

        // Utility function to parse param map and create a verbose text
        template <typename T>
        void setParam(ParamMap *map, std::string param_name, T* param, T param_default){
            // Setup verbose description
            std::string verbose_text("");
            ParamMap::iterator it = map->find(std::string("verbose_text"));
            if (it != map->end()){
                // Already have some verbose_text
                verbose_text = it->second + "\n";
            }
            verbose_text += param_name + ": ";

            // Parse param
            it = map->find(param_name);
            if (it != map->end()){
                std::istringstream iss(it->second);     // String conversion
                iss >> *param;
                verbose_text += it->second;
            } else {
                *param = param_default;
                std::ostringstream oss("");
                oss << param_default;
                verbose_text += oss.str() + " (default)";
            }
            (*map)[std::string("verbose_text")] = verbose_text;
        }

        // Throw an exception if module parametrization is invalid. Call this after module constructors.
        void assureParamsValid() {
            std::string error_message(""); // default error
            if (!checkParamsValid(&error_message)){
                throw std::invalid_argument("Invalid module parameters: " + error_message);
            }
        }

        // Allow modules to specify validity conditions. Return true if valid. Per default all params are valid.
        virtual bool checkParamsValid(std::string *error_message) { return true; }

        // whether to display the module creation info
        bool verbose_modules_;
    };

    // Base class to create modules. Setup a concrete factory for different parametrization methods.
    class ModuleFactory {
    public:
        virtual ~ModuleFactory() {}

        // Singleton accessor
        static ModuleFactory *Instance();

        // Module creation (this is the main accessor for new modules).
        template<class ModuleType>
        std::unique_ptr <ModuleType> createModule(const std::string &args, bool verbose){
            std::unique_ptr <ModuleType> result;
            Module::ParamMap param_map;
            if (!moduleCreationStep<ModuleType>(&param_map, &result, args, verbose)) {
                return nullptr;
            }
            moduleParametrizationStep(&param_map, result.get(), verbose);
            return result;
        }

        // Overload for modules that need a voxblox map (TrajectoryEvaluator, SensorModel)
        template<class ModuleType>
        std::unique_ptr <ModuleType> createModule(const std::string &args, bool verbose, std::shared_ptr<voxblox::EsdfServer> voxblox_ptr){
            std::unique_ptr <ModuleType> result;
            Module::ParamMap param_map;
            if (!moduleCreationStep<ModuleType>(&param_map, &result, args, verbose)) {
                return nullptr;
            }
            result->setVoxbloxPtr(voxblox_ptr);
            moduleParametrizationStep(&param_map, result.get(), verbose);
            return result;
        }

        // Overload for modules that need a parent pointer (EvaluatorUpdater, GeneratorUpdater)
        template<class ModuleType>
        std::unique_ptr <ModuleType> createModule(const std::string &args, bool verbose, Module* parent){
            std::unique_ptr <ModuleType> result;
            Module::ParamMap param_map;
            if (!moduleCreationStep<ModuleType>(&param_map, &result, args, verbose)) {
                return nullptr;
            }
            result->setParent(parent);
            moduleParametrizationStep(&param_map, result.get(), verbose);
            return result;
        }

        // Overload for modules that need both (TrajectoryGenerator)
        template<class ModuleType>
        std::unique_ptr <ModuleType> createModule(const std::string &args, bool verbose, std::shared_ptr<voxblox::EsdfServer> voxblox_ptr, Module* parent){
            std::unique_ptr <ModuleType> result;
            Module::ParamMap param_map;
            if (!moduleCreationStep<ModuleType>(&param_map, &result, args, verbose)) {
                return nullptr;
            }
            result->setVoxbloxPtr(voxblox_ptr);
            result->setParent(parent);
            moduleParametrizationStep(&param_map, result.get(), verbose);
            return result;
        }

        // Allow modules to register themselves to the factory using a static Registration struct
        template<class ModuleType>
        struct Registration{
            Registration(const std::string &module_name) {
                    ModuleMap::iterator it = getModuleMap()->find(module_name);
                    if(it == getModuleMap()->end()){
                        (*getModuleMap())[module_name] = &parseModuleMap<ModuleType>;
                    } else {
                        throw std::logic_error("ModuleFactory: Cannot register already existing module name '" + module_name + "'!");
                    }
                }
            };

    protected:
        // Module map type
        typedef std::map<std::string, Module*(*)()> ModuleMap;

        ModuleFactory() {}

        // Singleton instance
        static ModuleFactory *instance_;

        // Module Map instance
        static ModuleMap* module_map_;

        // Module Map accessor
        static ModuleMap *getModuleMap();

        // map parsing (to call module default constructors)
        template<class ModuleType>
        static Module* parseModuleMap() {
            return new ModuleType();
        }

        // Specify default types for modules that need such (e.g. doNothing modules)
        void getDefaultType(const std::type_index &module_index, std::string *type);

        // Two step base routine for creating modules (to allow members to be setup before parametrization)
        template <class ModuleType>
        bool moduleCreationStep(Module::ParamMap* map, std::unique_ptr <ModuleType> *result, const std::string &args, bool verbose) {
            std::string type = "NoDefaultTypeAvailable";
            getDefaultType(std::type_index(typeid(ModuleType)), &type);
            getParamMapAndType(map, &type, args);
            ModuleMap::iterator it = getModuleMap()->find(type);
            if(it == getModuleMap()->end()) {
                std::stringstream ss;
                ss << "Unknown " << typeid(ModuleType).name() << " type '" << type << "'.";
                printError(ss.str());
                return false;
            }
            ModuleType* module = dynamic_cast<ModuleType*>(it->second());
            module->verbose_modules_ = verbose;
            result->reset(module);
            return true;
        }

        bool moduleParametrizationStep(Module::ParamMap* map, Module *module, bool verbose);

        // Derived Factory responsibilities
        // Create a param map from the arg string (ROS, string, file, ...)
        virtual bool getParamMapAndType(Module::ParamMap *map, std::string *type, std::string args) = 0;

        // Modules add verbose param text to the map. Implement this if you want to print the result.
        virtual void printVerbose(const Module::ParamMap &map) {}

        // Implement this to print type errors
        virtual void printError(const std::string &message) {}
    };

    // Concrete factory for ros param server
    class ModuleFactoryROS : public ModuleFactory {
        friend ModuleFactory;

    public:
        ~ModuleFactoryROS() {}

    protected:
        ModuleFactoryROS() {}

        // Implement virtual methods
        bool getParamMapAndType(Module::ParamMap *map, std::string *type, std::string args);

        void printVerbose(const Module::ParamMap &map);

        void printError(const std::string &message);
    };

} // namepsace mav_active_3d_planning
#endif //MAV_ACTIVE_3D_PLANNING_MODULE_FACTORY_H
