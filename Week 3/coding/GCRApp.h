#ifndef GCR_APP_H
#define GCR_APP_H

#include <memory>
#include <SFML/Graphics.hpp>
#include <vector>
#include <string>
#include <cstdint>
#include "SpatialGraph.h"
#include "Observer.h"

struct MapLabel {
    std::string text;
    float x;
    float y;
    uint64_t id;
};

class GCRApp : public IObserver {
public:
    GCRApp(const SpatialGraph& graph, const std::vector<MapLabel>& rawLabels);
    void run();

    void onPathCalculated(const std::vector<uint64_t>& path, 
                         double executionTimeMs, 
                         double totalDistance,
                         std::string algorithmUsed) override;

    // --- NEW: Added public or private handler for parsing requests ---
    void processRoutingRequest();

private:
    // ==========================================
    // NEW: Floating Telemetry UI State Variables
    // ==========================================
    bool isMetricsPosInitialized = false; // Tracks if the box has been anchored to the destination yet
    sf::Vector2f metricsBoxPos;           // Tracks the current screen coordinates of the draggable box
    bool isDraggingMetrics = false;       // Tracks if the user is currently holding the box
    sf::Vector2f dragOffset;              // Prevents the box from snapping its top-left corner to the mouse

    // ==========================================
    // NEW: Routing Math Variables
    // ==========================================
    float routeDistancePixels = 0.0f;
    // User Input UI State
    std::string startInputStr = "";
    std::string endInputStr = "";
    bool isStartActive = true; 
    std::string errorMessage = "";
    bool hasRoutingError = false;

    // --- UPDATED TELEMETRY BENCHMARK METRICS ---
    struct AlgoMetrics {
        double latencyMs = 0.0;       // Execution duration profile
        size_t verticesTraversed = 0; // Settled nodes popped from min-heap queue
        size_t edgesTraversed = 0;    // Radial edge connections evaluated/relaxed
    };
    
    AlgoMetrics dijkstraData;
    AlgoMetrics astarData;
    bool hasBenchmarkResults = false;
    sf::Vector2f destinationPixelPos;

    // UI Render Elements
    sf::RectangleShape hudBackground;

    std::unique_ptr<sf::Text> hudText;
    sf::RectangleShape inputBackground;
    std::unique_ptr<sf::Text> inputTextOverlay;

    // Helper method to look up a road's closest node ID from its text string name
    uint64_t findClosestNodeByLocationName(const std::string& locationName);
    void processEvents();
    void update();
    void render();
    void buildMapGeometry();
    void buildLabels(const std::vector<MapLabel>& rawLabels);
    sf::Vector2f screenToLatLon(float screenX, float screenY);

    sf::RenderWindow window;
    sf::View cameraView;
    const SpatialGraph& mapGraph;
    
    sf::VertexArray mapGeometry; 
    sf::VertexArray pathGeometry;

    sf::Font mapFont;
    std::vector<MapLabel> mapLabels;          // Holds database strings & nodeIds
    std::vector<sf::Text> mapLabelDrawables; // Holds visual SFML engine layouts

    bool isDragging = false;
    sf::Vector2f oldMousePos;

    double minLat = 90.0, maxLat = -90.0;
    double minLon = 180.0, maxLon = -180.0;

    uint64_t startNodeId = 0;
    uint64_t endNodeId = 0;
    
    sf::CircleShape startMarker;
    sf::CircleShape endMarker;
};

#endif // GCR_APP_H
