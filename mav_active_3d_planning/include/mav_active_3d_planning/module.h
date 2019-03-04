#ifndef MAV_ACTIVE_3D_PLANNING_MODULE_H
#define MAV_ACTIVE_3D_PLANNING_MODULE_H

#include <string>
#include <sstream>
#include <map>

namespace mav_active_3d_planning {

    // Base class for all modules, provides creation functionalities
    class Module{
        friend class ModuleFactory;

    public:
        typedef std::map<std::string, std::string> ParamMap;
        virtual ~Module() {}

    protected:
        Module() {}

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
                verbose_text += std::to_string(param_default) + " (default)";
            }
            (*map)[std::string("verbose_text")] = verbose_text;
        }
    };

}; // namepsace mav_active_3d_planning
#endif //MAV_ACTIVE_3D_PLANNING_MODULE_H
