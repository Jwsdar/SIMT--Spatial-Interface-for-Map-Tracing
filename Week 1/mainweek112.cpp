#include <iostream>
#include <vector>
#include <algorithm>
#include "MapAdapter.h"

int main() {
    OSMXmlAdapter adapter;
    std::string path = "C:\\Users\\jawad\\OneDrive\\Documents\\GitHub\\SIMT- Spatial Interface for Map Tracing\\Week 1\\map.osm";
    
    if (adapter.loadMapFile(path)) {
        // Step A: Extract physical coordinates and bounding box attributes
        adapter.extractBounds();
        std::vector<Node> mapNodes = adapter.extractNodes();
        
        BoundingBox box = adapter.getBounds();
        std::cout << "--- MAP BOUNDARY ATTRIBUTES ---" << std::endl;
        std::cout << "Bounding Limits: Lat[" << box.minLat << " to " << box.maxLat 
                  << "] | Lon[" << box.minLon << " to " << box.maxLon << "]\n\n";

        // Step B: Target specifically your designated NUTECH node ID
        uint64_t targetNodeId = 4347907449;
        auto it = std::find_if(mapNodes.begin(), mapNodes.end(), [targetNodeId](const Node& n) {
            return n.id == targetNodeId;
        });

        if (it != mapNodes.end()) {
            // Define targeted SFML output window dimension sizes
            float windowWidth = 1024.0f;
            float windowHeight = 768.0f;

            // Step C: Execute WBS 1.1.2 Transformation Engine
            ScreenPoint pixel = adapter.projectToScreen(it->latitude, it->longitude, windowWidth, windowHeight);

            std::cout << "--- WBS 1.1.2 COORDINATE NORMALIZATION REPORT ---" << std::endl;
            std::cout << "Node ID        : " << it->id << "\n";
            std::cout << "Landmark Area  : " << it->name << "\n";
            std::cout << "Global Lat/Lon : (" << it->latitude << ", " << it->longitude << ")\n";
            std::cout << "SFML Pixel X/Y : (" << pixel.x << "px, " << pixel.y << "px)\n";
            std::cout << "=================================================\n";
        }
    }
    system("pause");
    
    return 0;
}