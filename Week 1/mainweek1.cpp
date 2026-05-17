#include <iostream>
#include <vector>
#include "MapAdapter.h"

int main() {
    OSMXmlAdapter adapter;
    std::string path = "C:\\Users\\jawad\\OneDrive\\Documents\\GitHub\\SIMT- Spatial Interface for Map Tracing\\Week 1\\map.osm";
    
    if (adapter.loadMapFile(path)) {
        std::vector<Node> mapNodes = adapter.extractNodes();
        
        // Target specifically the node inside the NUTECH bounding area layout
        uint64_t targetNodeId = 4347907449;
        
        // Call the new display function
        adapter.displayNodeDetails(mapNodes, targetNodeId);
    }
    system("pause");
    return 0;
}