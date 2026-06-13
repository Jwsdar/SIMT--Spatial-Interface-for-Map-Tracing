#include "PathfindingStrategy.h"
#include "SpatialGraph.h"
#include <queue>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <functional>

// Helper structure to reconstruct paths from destination parent-links back to origin
// Helper structure to reconstruct paths from destination parent-links back to origin
namespace {
    std::vector<uint64_t> reconstructPath(const std::unordered_map<uint64_t, uint64_t>& parentMap, 
                                          uint64_t targetId, 
                                          uint64_t startNodeId) {
        std::vector<uint64_t> path;
        uint64_t currentId = targetId;
        path.push_back(currentId);
        
        // Loop backward until we hit our designated starting point anchor
        while (currentId != startNodeId) {
            auto it = parentMap.find(currentId);
            if (it == parentMap.end()) {
                // Network graph failure fallback check
                return std::vector<uint64_t>(); 
            }
            currentId = it->second;
            path.push_back(currentId);
        }
        
        // Reverse the array sequence so it runs correctly from start to finish
        std::reverse(path.begin(), path.end());
        return path;
    }
}


// WBS 1.3.1: Dijkstra's Routing Solver (Instrumented)


std::vector<uint64_t> DijkstraStrategy::findPath(const SpatialGraph& graph, 
                                                 uint64_t startNodeId, 
                                                 uint64_t endNodeId,
                                                 size_t& verticesTraversed,
                                                 size_t& edgesTraversed) {
    // Reset performance tracking metrics
    verticesTraversed = 0;
    edgesTraversed = 0;

    // Edge case: Verify both nodes exist in the spatial graph
    if (!graph.hasNode(startNodeId) || !graph.hasNode(endNodeId)) {
        return std::vector<uint64_t>(); // Returns empty path
    }
    
    // Min-Priority Queue tracking: std::pair<cumulative_cost g(n), current_node_id>
    std::priority_queue<std::pair<double, uint64_t>, 
                        std::vector<std::pair<double, uint64_t>>, 
                        std::greater<std::pair<double, uint64_t>>> pq;

    // Track minimum cost g(n) reached for each node ID
    std::unordered_map<uint64_t, double> distMap;
    // Map tracking route backtracking history
    std::unordered_map<uint64_t, uint64_t> parentMap;

    // Initialize origin
    distMap[startNodeId] = 0.0;
    pq.push({0.0, startNodeId});

    while (!pq.empty()) {
        auto [currentCost, currentId] = pq.top();
        pq.pop();

        // Metric Increment: A node has been settled/processed out of the queue
        verticesTraversed++;

        // Short-circuit: If destination is extracted, the absolute shortest path is locked in
        if (currentId == endNodeId) {
            // FIXED: Passing startNodeId to act as the reverse path trace termination loop ceiling
            return reconstructPath(parentMap, endNodeId, startNodeId);
        }

        // Lazy deletion optimization check: Skip if a cheaper route was already processed
        if (currentCost > distMap[currentId]) {
            continue;
        }

        // Radial exploration of neighboring highway edges
        for (const auto& edge : graph.getNeighbors(currentId)) {
            // Metric Increment: An edge/road connection is evaluated and traversed
            edgesTraversed++;

            uint64_t neighborId = edge.targetNodeId;
            double branchCost = currentCost + edge.weight;

            // Relaxation Step
            if (distMap.find(neighborId) == distMap.end() || branchCost < distMap[neighborId]) {
                distMap[neighborId] = branchCost;
                parentMap[neighborId] = currentId;
                pq.push({branchCost, neighborId});
            }
        }
    }

    return std::vector<uint64_t>(); // Empty vector indicates destination is completely unreachable
}


// WBS 1.3.2: A* Heuristic Routing Solver (Instrumented)

std::vector<uint64_t> AStarStrategy::findPath(const SpatialGraph& graph, 
                                             uint64_t startNodeId, 
                                             uint64_t endNodeId,
                                             size_t& verticesTraversed,
                                             size_t& edgesTraversed) {
    // Reset performance tracking metrics
    verticesTraversed = 0;
    edgesTraversed = 0;

    if (!graph.hasNode(startNodeId) || !graph.hasNode(endNodeId)) {
        return std::vector<uint64_t>();
    }

    const Vertex& targetVertex = graph.getVertex(endNodeId);

    // Min-Priority Queue tracking: std::pair<total_estimated_cost f(n), current_node_id>
    std::priority_queue<std::pair<double, uint64_t>, 
                        std::vector<std::pair<double, uint64_t>>, 
                        std::greater<std::pair<double, uint64_t>>> pq;

    // Separate track for accurate cumulative cost g(n)
    std::unordered_map<uint64_t, double> gCostMap;
    std::unordered_map<uint64_t, uint64_t> parentMap;

    // Initialize origin node parameters
    gCostMap[startNodeId] = 0.0;
    
    // Initial heuristic h(n) from start to destination via Haversine Math Engine (WBS 1.2.2)
    const Vertex& startVertex = graph.getVertex(startNodeId);
    double initialH = HaversineEngine::calculateDistance(
        startVertex.latitude, startVertex.longitude,
        targetVertex.latitude, targetVertex.longitude
    );

    pq.push({initialH, startNodeId}); // f(n) = g(n) + h(n) -> 0.0 + initialH

    while (!pq.empty()) {
        auto [estimatedTotalCost, currentId] = pq.top();
        pq.pop();

        // Metric Increment: A node has been popped/settled for evaluation
        verticesTraversed++;

        if (currentId == endNodeId) {
            // FIXED: Passing startNodeId to act as the reverse path trace termination loop ceiling
            return reconstructPath(parentMap, endNodeId, startNodeId);
        }

        double currentG = gCostMap[currentId];

        // Process neighbors
        for (const auto& edge : graph.getNeighbors(currentId)) {
            // Metric Increment: An edge relaxation check occurs
            edgesTraversed++;

            uint64_t neighborId = edge.targetNodeId;
            double tentativeG = currentG + edge.weight;

            if (gCostMap.find(neighborId) == gCostMap.end() || tentativeG < gCostMap[neighborId]) {
                gCostMap[neighborId] = tentativeG;
                parentMap[neighborId] = currentId;

                // Evaluate spatial remaining distance heuristic h(n) on the fly
                const Vertex& neighborVertex = graph.getVertex(neighborId);
                double h = HaversineEngine::calculateDistance(
                    neighborVertex.latitude, neighborVertex.longitude,
                    targetVertex.latitude, targetVertex.longitude
                );

                double f = tentativeG + h; // Core A* weight mapping: f(n) = g(n) + h(n)
                pq.push({f, neighborId});
            }
        }
    }

    return std::vector<uint64_t>(); // Empty vector means destination unreachable
}