#ifndef PATHFINDING_STRATEGY_H
#define PATHFINDING_STRATEGY_H

#include <vector>
#include <cstdint>
#include "SpatialGraph.h" // Provides the optimized Adjacency List Graph (WBS 1.2.1)


// WBS 1.3: Strategy Pattern Algorithmic Interface

class PathfindingStrategy {
public:
    virtual ~PathfindingStrategy() = default;

    /*
     * @brief Core routing interface method shared by all path solvers.
     * @param graph The compiled SpatialGraph containing intersections and weights.
     * @param startNodeId The 64-bit OSM ID of the origin/takeoff position.
     * @param endNodeId The 64-bit OSM ID of the destination/landing target.
     * @return std::vector<uint64_t> Ordered sequence of node IDs from start to end.
     * Returns an empty vector if path is blocked.
     */

    virtual std::vector<uint64_t> findPath(const SpatialGraph& graph, 
                                           uint64_t startNodeId, 
                                           uint64_t endNodeId) = 0;
};


// WBS 1.3.1: Dijkstra's Routing Solver

class DijkstraStrategy : public PathfindingStrategy {
public:
    DijkstraStrategy() = default;

    /*
     * @brief Computes the absolute shortest path using radial exploration.
     * Expands tracking frontiers strictly based on cumulative edge weights g(n)
     * using a standard min-priority queue (std::priority_queue).
     */
    std::vector<uint64_t> findPath(const SpatialGraph& graph, 
                                   uint64_t startNodeId, 
                                   uint64_t endNodeId) override;
};


// WBS 1.3.2: A* Heuristic Routing Solver

class AStarStrategy : public PathfindingStrategy {
public:
    AStarStrategy() = default;

    /*
     * @brief Accelerates path calculation by introducing spatial heuristics.
     * Minimizes f(n) = g(n) + h(n), where h(n) represents the remaining aerial
     * distance computed on-the-fly via the Haversine Math Engine (WBS 1.2.2).
     */
    std::vector<uint64_t> findPath(const SpatialGraph& graph, 
                                   uint64_t startNodeId, 
                                   uint64_t endNodeId) override;
};

#endif // PATHFINDING_STRATEGY_H