#define _USE_MATH_DEFINES
#include "SpatialGraph.h"
#include <iostream>
#include <cmath>


// WBS 1.2.2: Haversine Math Engine Implementation
double HaversineEngine::calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    // Earth's mean radius in meters
    const double EARTH_RADIUS_METERS = 6371000.0;

    // Convert degrees to radians
    double rLat1 = lat1 * M_PI / 180.0;
    double rLon1 = lon1 * M_PI / 180.0;
    double rLat2 = lat2 * M_PI / 180.0;
    double rLon2 = lon2 * M_PI / 180.0;

    // Differences in coordinates
    double dLat = rLat2 - rLat1;
    double dLon = rLon2 - rLon1;

    // Haversine core formula computation
    double a = std::sin(dLat / 2.0) * std::sin(dLat / 2.0) +
               std::cos(rLat1) * std::cos(rLat2) *
               std::sin(dLon / 2.0) * std::sin(dLon / 2.0);
               
    double c = 2.0 * std::asin(std::sqrt(a));
    
    return EARTH_RADIUS_METERS * c; // Returns absolute distance in meters
}


// WBS 1.2.1: Spatial Adjacency List Graph Implementation

void SpatialGraph::insertNode(const Node& osmNode) {
    // Avoid double insertions; build vertex frame with an empty adjacency tracking layout
    if (adjacencyMap.find(osmNode.id) == adjacencyMap.end()) {
        Vertex v;
        v.id = osmNode.id;
        v.name = osmNode.name;
        v.latitude = osmNode.latitude;
        v.longitude = osmNode.longitude;
        adjacencyMap[osmNode.id] = v;
    }
}

void SpatialGraph::addEdge(uint64_t sourceId, uint64_t targetId, double weight) {
    // In OpenStreetMap routing, we can safely only map edges if both nodes structurally exist
    if (adjacencyMap.find(sourceId) != adjacencyMap.end() && 
        adjacencyMap.find(targetId) != adjacencyMap.end()) {
        
        Edge e;
        e.targetNodeId = targetId;
        e.weight = weight;
        
        adjacencyMap[sourceId].adjacencyList.push_back(e);
    }
}

// Compiles complete OpenStreetMap vector sequence into an active traversal memory block
void SpatialGraph::compileGraph(const std::vector<Node>& nodes, const std::vector<Way>& ways) {
    // 1. Ingest all vertices into our hash lookup profile framework
    for (const auto& node : nodes) {
        insertNode(node);
    }

    // 2. Parse structural road networks to link active paths
    for (const auto& way : ways) {
        // Run sequentially through adjacent sequential reference ids
        for (size_t i = 0; i < way.node_refs.size() - 1; ++i) {
            uint64_t u = way.node_refs[i];
            uint64_t v = way.node_refs[i + 1];

            // Verify both coordinate components reside inside the active subset map boundaries
            if (hasNode(u) && hasNode(v)) {
                const Vertex& nodeU = getVertex(u);
                const Vertex& nodeV = getVertex(v);

                // Execute WBS 1.2.2 Haversine formula weight evaluation
                double physicalDistance = HaversineEngine::calculateDistance(
                    nodeU.latitude, nodeU.longitude,
                    nodeV.latitude, nodeV.longitude
                );

                // Add edge sequentially (Assuming bidirectional roads for base prototype)
                addEdge(u, v, physicalDistance);
                addEdge(v, u, physicalDistance);
            }
        }
    }
}

uint64_t SpatialGraph::findNearestRoutingNode(double lat, double lon) const {
    uint64_t closestId = 0;
    double minDistance = std::numeric_limits<double>::max();

    for (const auto& pair : adjacencyMap) {
        // Only target nodes that are actually part of the road network (have edges)
        if (!pair.second.adjacencyList.empty()) {
            double dist = HaversineEngine::calculateDistance(lat, lon, pair.second.latitude, pair.second.longitude);
            if (dist < minDistance) {
                minDistance = dist;
                closestId = pair.first;
            }
        }
    }
    return closestId;
}

bool SpatialGraph::hasNode(uint64_t nodeId) const {
    return adjacencyMap.find(nodeId) != adjacencyMap.end();
}

const Vertex& SpatialGraph::getVertex(uint64_t nodeId) const {
    return adjacencyMap.at(nodeId); // Uses .at() to enforce read-only safety lookups
}

const std::vector<Edge>& SpatialGraph::getNeighbors(uint64_t nodeId) const {
    return adjacencyMap.at(nodeId).adjacencyList;
}

size_t SpatialGraph::getNodeCount() const {
    return adjacencyMap.size();
}

void SpatialGraph::displayGraphMetrics() const {
    size_t edgeCount = 0;
    for (const auto& pair : adjacencyMap) {
        edgeCount += pair.second.adjacencyList.size();
    }
    std::cout << "\n======================================" << std::endl;
    std::cout << "     WBS 1.2: CORE GRAPH COMPLETED    " << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "Total Active Map Nodes (Vertices) : " << adjacencyMap.size() << std::endl;
    std::cout << "Total Routed Connections (Edges) : " << edgeCount << std::endl;
    std::cout << "Memory Optimization Target        : STL Hash Map Adjacency List Verified" << std::endl;
    std::cout << "======================================\n" << std::endl;
}