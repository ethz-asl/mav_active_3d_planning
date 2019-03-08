#ifndef MAV_ACTIVE_3D_PLANNING_MODULE_H
#define MAV_ACTIVE_3D_PLANNING_MODULE_H

#include <string>
#include <sstream>
#include <map>
#include <stdexcept>

namespace mav_active_3d_planning {

    // Base class for all modules, provides creation functionalities
    class Module{
        friend class ModuleFactory;

    public:
        typedef std::map<std::string, std::string> ParamMap;
        virtual ~Module() {}

    protected:
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

}; // namepsace mav_active_3d_planning
#endif //MAV_ACTIVE_3D_PLANNING_MODULE_H
