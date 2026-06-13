#include <iostream>
#include <vector>
#include <chrono> // For measuring path execution runtime profiles
#include "MapAdapter.h"
#include "SpatialGraph.h"
#include "PathfindingStrategy.h"

// Helper function to calculate the total path length in meters using edge weights
double computeTotalPathDistance(const SpatialGraph& graph, const std::vector<uint64_t>& path) {
    if (path.size() < 2) return 0.0;
    
    double totalDistance = 0.0;
    
    // Step through the path nodes sequentially
    for (size_t i = 0; i < path.size() - 1; ++i) {
        uint64_t currentId = path[i];
        uint64_t nextId = path[i + 1];
        
        // Look up the exact weight (Haversine meters) assigned to this edge in WBS 1.2.1
        const auto& neighbors = graph.getNeighbors(currentId);
        bool edgeFound = false;
        
        for (const auto& edge : neighbors) {
            if (edge.targetNodeId == nextId) {
                totalDistance += edge.weight;
                edgeFound = true;
                break;
            }
        }
        
        // Fallback: If an edge anomaly happens, compute straight-line distance directly
        if (!edgeFound) {
            const Vertex& v1 = graph.getVertex(currentId);
            const Vertex& v2 = graph.getVertex(nextId);
            totalDistance += HaversineEngine::calculateDistance(v1.latitude, v1.longitude, v2.latitude, v2.longitude);
        }
    }
    
    return totalDistance;
}

int main() {
    OSMXmlAdapter adapter;
    std::string path = "C:\\Users\\jawad\\OneDrive\\Documents\\GitHub\\SIMT- Spatial Interface for Map Tracing\\Week 1\\map.osm";
    
    std::cout << "Step 1: Loading Map Clip...\n";
    if (!adapter.loadMapFile(path)) return -1;

    adapter.extractBounds();
    std::vector<Node> extractedNodes = adapter.extractNodes();
    std::vector<Way> extractedWays = adapter.extractWays();

    std::cout << "Step 2: Building Adjacency List Graph Engine...\n";
    SpatialGraph mapGraph;
    mapGraph.compileGraph(extractedNodes, extractedWays);
    mapGraph.displayGraphMetrics();

    // Picking simulation flight waypoints within the Islamabad/NUTECH clip area
    std::cout << "Step 3: Simulating Flight Waypoints...\n";
    uint64_t startPoint = mapGraph.findNearestRoutingNode(33.62590886108029, 73.01137229247036);
    uint64_t endPoint = mapGraph.findNearestRoutingNode(33.63074648494975, 73.0179110004097);

    if (startPoint == 0 || endPoint == 0) {
        std::cerr << "Error: Waypoint targets could not be mapped to the routing network.\n";
        return -1;
    }

    // Define context strategy pointer base type (Strategy Pattern)
    PathfindingStrategy* pathfinder = nullptr;

    // --- TEST RUN A: DIJKSTRA RADIAL EXPLORATION ---
    pathfinder = new DijkstraStrategy();
    auto startClock = std::chrono::high_resolution_clock::now();
    std::vector<uint64_t> dijkstraPath = pathfinder->findPath(mapGraph, startPoint, endPoint);
    auto stopClock = std::chrono::high_resolution_clock::now();
    auto dijkstraDuration = std::chrono::duration_cast<std::chrono::microseconds>(stopClock - startClock).count();
    double dijkstraDistance = computeTotalPathDistance(mapGraph, dijkstraPath);
    delete pathfinder;

    // --- TEST RUN B: A* HEURISTIC ACCELERATED SEARCH ---
    pathfinder = new AStarStrategy();
    startClock = std::chrono::high_resolution_clock::now();
    std::vector<uint64_t> aStarPath = pathfinder->findPath(mapGraph, startPoint, endPoint);
    stopClock = std::chrono::high_resolution_clock::now();
    auto aStarDuration = std::chrono::duration_cast<std::chrono::microseconds>(stopClock - startClock).count();
    double aStarDistance = computeTotalPathDistance(mapGraph, aStarPath);
    delete pathfinder;

    // --- PRINT DETAILED COMPARATIVE METRICS REPORT ---
    std::cout << "\n=================================================" << std::endl;
    std::cout << "        WBS 1.3: ROUTING ROUTINE METRICS         " << std::endl;
    std::cout << "=================================================" << std::endl;
    std::cout << "Origin Flight Node ID      : " << startPoint << std::endl;
    std::cout << "Destination Target ID      : " << endPoint << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;
    std::cout << "DIJKSTRA'S METHOD SOLVER:" << std::endl;
    std::cout << " -> Extracted Path Node Count: " << dijkstraPath.size() << " steps" << std::endl;
    std::cout << " -> Total Physical Distance  : " << dijkstraDistance << " meters (" << (dijkstraDistance / 1000.0) << " km)" << std::endl;
    std::cout << " -> Internal Execution Latency: " << dijkstraDuration << " microseconds" << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;
    std::cout << "A* SPATIAL HEURISTIC SOLVER:" << std::endl;
    std::cout << " -> Extracted Path Node Count: " << aStarPath.size() << " steps" << std::endl;
    std::cout << " -> Total Physical Distance  : " << aStarDistance << " meters (" << (aStarDistance / 1000.0) << " km)" << std::endl;
    std::cout << " -> Internal Execution Latency: " << aStarDuration << " microseconds" << std::endl;
    std::cout << "=================================================\n" << std::endl;
    system("pause");
    return 0;
}