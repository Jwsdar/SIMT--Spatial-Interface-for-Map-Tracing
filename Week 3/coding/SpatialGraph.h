#ifndef SPATIAL_GRAPH_H
#define SPATIAL_GRAPH_H

#include <vector>
#include <unordered_map>
#include <string>
#include "MapAdapter.h" // Utilizes Week 1 structures (Node, Way)

// WBS 1.2.1: Graph Edge Structure representation
struct Edge {
    uint64_t targetNodeId;
    double weight; // Physical distance in meters (computed via Haversine)
};

// WBS 1.2.1: Internal Vertex Structure to accommodate WBS 1.3 Heuristic lookups
struct Vertex {
    uint64_t id;
    std::string name;
    double latitude;
    double longitude;
    std::vector<Edge> adjacencyList; // Outgoing road links
};

// WBS 1.2.2: Haversine Math Engine Namespace/Utility
class HaversineEngine {
public:
    // Computes the absolute surface distance between two coordinate pairs in meters
    static double calculateDistance(double lat1, double lon1, double lat2, double lon2);
};

// WBS 1.2.1: Performance-Optimized Spatial Graph Class
class SpatialGraph {
private:
    // Core structure: Hash map matching unique 64-bit OSM ID to our internal Vertex representation
    std::unordered_map<uint64_t, Vertex> adjacencyMap;

public:
    // Allows modification of internal vertex data parameters during parsing pipeline stages
    std::unordered_map<uint64_t, Vertex>& getAdjacencyMapMutable() {
        return adjacencyMap;
    }
    // Test helper: Add a vertex
    void addTestVertex(uint64_t id, double lat, double lon) {
        // Initializes the vertex with the given ID, coordinates, and an empty edge list
        adjacencyMap[id] = Vertex{id, "TestNode", lat, lon, {}};
    }

    // Test helper: Create a one-way connection (edge) between two nodes
    void addTestEdge(uint64_t fromId, uint64_t toId) {
        // Appends an Edge to the source vertex's adjacency list
        adjacencyMap[fromId].adjacencyList.push_back(Edge{toId, 1.0}); // 1.0 is a dummy weight
    }
    // Add this public getter function:
    const std::unordered_map<uint64_t, Vertex>& getAdjacencyMap() const {
        return adjacencyMap;
    }
    SpatialGraph() = default;

    // Mutator Methods
    void insertNode(const Node& osmNode);
    void addEdge(uint64_t sourceId, uint64_t targetId, double weight);
    
    // Bulk Pipeline compiler (Consumes your extracted Week 1 data streams)
    void compileGraph(const std::vector<Node>& nodes, const std::vector<Way>& ways);


    // Finds the closest node that actually belongs to the highway routing network
    uint64_t findNearestRoutingNode(double lat, double lon) const;
    
    // Accessor Methods (Critical for WBS 1.3 Pathfinding engines)
    bool hasNode(uint64_t nodeId) const;
    const Vertex& getVertex(uint64_t nodeId) const;
    const std::vector<Edge>& getNeighbors(uint64_t nodeId) const;
    size_t getNodeCount() const;
    
    // Debug utility
    void displayGraphMetrics() const;
};

#endif // SPATIAL_GRAPH_H