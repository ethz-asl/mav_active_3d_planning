#include <active_3d_planning/data/visualization_markers.h>

namespace active_3d_planning {

    void VisualizationMarkers::addMarker(const VisualizationMarker &marker) {
        markers.push_back(marker);
    }

    std::vector<VisualizationMarker>&VisualizationMarkers::getMarkers(){
        return markers;
    }

    int VisualizationMarkers::getNextVisualizationId() {
        if (markers.empty()) {
            return 0;
        } else {
            return markers.back().id + 1;
        }
    }

} // namespace active_3d_planning
