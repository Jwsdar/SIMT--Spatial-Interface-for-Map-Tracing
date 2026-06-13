#include <iostream>
#include <vector>
#include "MapAdapter.h" // Utilizes Week 1 structures (Node, Way)
#include "SpatialGraph.h"

int main() {
    OSMXmlAdapter adapter;
    std::string path = "C:\\Users\\jawad\\OneDrive\\Documents\\GitHub\\SIMT- Spatial Interface for Map Tracing\\Week 1\\map.osm";
    
    std::cout << "Initializing SIMT Ingestion Component...\n";
    if (adapter.loadMapFile(path)) {
        // Step 1: Extract Base Vectors (Week 1 / WBS 1.1)
        adapter.extractBounds();
        std::vector<Node> extractedNodes = adapter.extractNodes();
        std::vector<Way> extractedWays = adapter.extractWays();

        // Step 2: Ingest into Graph Subsystem (Week 2 / WBS 1.2)
        SpatialGraph mapGraph;
        std::cout << "Compiling Spatial Graph Network via Haversine Calculations...\n";
        mapGraph.compileGraph(extractedNodes, extractedWays);

        // Step 3: Print Status Metrics Report
        mapGraph.displayGraphMetrics();
        
        uint64_t ipNode;

        // Step 4: Input Node:
        std::cout << "Input Node of Location Check Required: " << std::endl;
        std::cin >> ipNode;


        // Step 5: Validation Check for future WBS 1.3 Strategy Routing Lookups
        uint64_t landmarkNodeId = ipNode;
        if (mapGraph.hasNode(landmarkNodeId)) {
            const Vertex& landmark = mapGraph.getVertex(landmarkNodeId);
            std::cout << "Targeting Landmark: " << landmark.name << "\n";
            
            // Snap to the actual road network!
            uint64_t snappedRoadNodeId = mapGraph.findNearestRoutingNode(landmark.latitude, landmark.longitude);
            
            if (snappedRoadNodeId != 0) {
                const Vertex& roadNode = mapGraph.getVertex(snappedRoadNodeId);
                std::cout << "-> Snapped to Nearest Drivable Node ID: " << snappedRoadNodeId << "\n";
                std::cout << "-> Distance to Road: " 
                        << HaversineEngine::calculateDistance(landmark.latitude, landmark.longitude, roadNode.latitude, roadNode.longitude) 
                        << " meters.\n";
                std::cout << "-> Outgoing Adjacent Highway Links Count: " << mapGraph.getNeighbors(snappedRoadNodeId).size() << "\n";
            }
        }
    }
    system("pause");
    return 0;
}