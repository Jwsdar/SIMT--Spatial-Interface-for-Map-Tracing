#ifndef MAP_ADAPTER_H
#define MAP_ADAPTER_H

#include <string>
#include <vector>
#include "tinyxml2.h"

// Core geographic data structures
struct Node {
    uint64_t id;
    std::string name;
    double latitude;
    double longitude;
};

struct Way {
    uint64_t id;
    std::vector<uint64_t> node_refs;
};

// Added for WBS 1.1.2: Normalized 2D Screen Coordinate Vector
struct ScreenPoint {
    float x;
    float y;
};

// Added for WBS 1.1.2: Bounding Box Boundaries
struct BoundingBox {
    double minLat;
    double maxLat;
    double minLon;
    double maxLon;
};

// Abstract Adapter Interface (WBS 1.1.1)
class MapDataAdapter {
public:
    virtual ~MapDataAdapter() = default;
    virtual bool loadMapFile(const std::string& filepath) = 0;
    virtual std::vector<Node> extractNodes() = 0;
    virtual std::vector<Way> extractWays() = 0;
};

// Concrete XML Adapter implementation
class OSMXmlAdapter : public MapDataAdapter {
private:
    tinyxml2::XMLDocument xmlDoc;
    bool isLoaded;

    // WBS 1.1.2 Internal State: The map file's explicit boundary coordinates
    BoundingBox mapBounds;

public:
    OSMXmlAdapter();
    bool loadMapFile(const std::string& filepath) override;
    std::vector<Node> extractNodes() override;
    std::vector<Way> extractWays() override;
    // Added: Targeted Node Inspector Function
    void displayNodeDetails(const std::vector<Node>& nodes, uint64_t targetId);

    // Added for WBS 1.1.2: Engine Interface methods
    bool extractBounds(); // Parses the <bounds> tag from the XML file
    BoundingBox getBounds() const { return mapBounds; }
    
    // Normalizes global spatial nodes to fit your exact SFML viewport dimensions
    ScreenPoint projectToScreen(double lat, double lon, float windowWidth, float windowHeight);


};

#endif // MAP_ADAPTER_H