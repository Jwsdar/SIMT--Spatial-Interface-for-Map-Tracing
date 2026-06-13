#pragma once
#include <vector>
#include <cstdint>
#include <string>

// WBS 1.4.2: Observer Pattern Foundation
class IObserver {
public:
    virtual ~IObserver() = default;
    
    // Called by the Pathfinding Engine when a route is successfully calculated
    virtual void onPathCalculated(const std::vector<uint64_t>& path, 
                                  double executionTimeMs, 
                                  double totalDistance,
                                  std::string algorithmUsed) = 0;
};