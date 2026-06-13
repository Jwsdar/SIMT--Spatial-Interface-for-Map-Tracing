#include <iostream>
#include <vector>
#include <string>
#include <exception>
#include <unordered_map> 
#include <limits> 
#include <cmath>
#include "SpatialGraph.h"
#include "GCRApp.h"
#include "MapAdapter.h"

int main() {
    std::cout << "=======================================================" << std::endl;
    std::cout << "   SIMT - Ground Control Resource Platform Engine Initializing  " << std::endl;
    std::cout << "=======================================================\n" << std::endl;

    try {
        SpatialGraph realGraph;
        OSMXmlAdapter osmAdapter;

        std::string osmFilePath = "map.osm"; 
        std::cout << "[PIPELINE] Loading OSM File: " << osmFilePath << "..." << std::endl;
        
        if (!osmAdapter.loadMapFile(osmFilePath)) {
            std::cerr << "CRITICAL ERROR: Failed to load target map data file: " << osmFilePath << std::endl;
            return 1;
        }

        if (!osmAdapter.extractBounds()) {
            std::cerr << "[WARNING] Missing <bounds> tags in XML. Screen scaling may distort." << std::endl;
        }

        std::cout << "[PIPELINE] Extracting geographical datasets..." << std::endl;
        std::vector<Node> osmNodes = osmAdapter.extractNodes();
        std::vector<Way> osmWays = osmAdapter.extractWays();

        std::cout << "   -> Target Dataset contains: " << osmNodes.size() << " Nodes." << std::endl;
        std::cout << "   -> Target Dataset contains: " << osmWays.size() << " Ways." << std::endl;

        struct LabelCentroid {
            float sumX = 0.f;
            float sumY = 0.f;
            int count = 0;
            uint64_t standardNodeId = 0;
        };
        std::unordered_map<std::string, LabelCentroid> labelAggregator;

        std::cout << "[PIPELINE] Transforming global spatial coordinates to 2D screen space..." << std::endl;
        for (const auto& node : osmNodes) {
            ScreenPoint screenPt = osmAdapter.projectToScreen(node.latitude, node.longitude, 1280.f, 720.f);

            // Storing screenPt.y as latitude, screenPt.x as longitude
            realGraph.addTestVertex(node.id, screenPt.y, screenPt.x);

            if (!node.name.empty()) {
                realGraph.getAdjacencyMapMutable()[node.id].name = node.name;
                labelAggregator[node.name].sumX += screenPt.x;
                labelAggregator[node.name].sumY += screenPt.y;
                labelAggregator[node.name].count++;
                labelAggregator[node.name].standardNodeId = node.id;
            }
        }

        std::cout << "[PIPELINE] Mapping grid topology network links..." << std::endl;
        size_t linkCount = 0;
        std::unordered_map<uint64_t, bool> activeRoadNodes;

        for (const auto& way : osmWays) {
            for (size_t i = 0; i < way.node_refs.size() - 1; ++i) {
                uint64_t fromNode = way.node_refs[i];
                uint64_t toNode = way.node_refs[i + 1];

                realGraph.addTestEdge(fromNode, toNode);
                realGraph.addTestEdge(toNode, fromNode);
                linkCount += 2;

                activeRoadNodes[fromNode] = true;
                activeRoadNodes[toNode] = true;
            }
        }
        std::cout << "   -> Structural edge parsing complete. Generated " << linkCount << " road segments." << std::endl;

        // --- NEAREST-NEIGHBOR ROUTING INSURANCE FILTER ---
        std::cout << "[PIPELINE] Processing and snapping labels to active road grid..." << std::endl;
        std::vector<MapLabel> mapLabelsToRender;
        const auto& graphMap = realGraph.getAdjacencyMap();

        for (const auto& [name, data] : labelAggregator) {
            float centerX = data.sumX / static_cast<float>(data.count);
            float centerY = data.sumY / static_cast<float>(data.count);
            
            uint64_t routingNodeId = data.standardNodeId;

            if (activeRoadNodes.find(routingNodeId) == activeRoadNodes.end()) {
                float minDistanceSq = std::numeric_limits<float>::max();
                uint64_t nearestStreetId = routingNodeId;

                if (graphMap.find(routingNodeId) != graphMap.end()) {
                    const auto& targetVertex = graphMap.at(routingNodeId);

                    for (const auto& [roadNodeId, _] : activeRoadNodes) {
                        if (graphMap.find(roadNodeId) != graphMap.end()) {
                            const auto& roadVertex = graphMap.at(roadNodeId);
                            
                            float dx = roadVertex.longitude - targetVertex.longitude; 
                            float dy = roadVertex.latitude - targetVertex.latitude; 
                            float distSq = (dx * dx) + (dy * dy);

                            if (distSq < minDistanceSq) {
                                minDistanceSq = distSq;
                                nearestStreetId = roadNodeId;
                            }
                        }
                    }
                    routingNodeId = nearestStreetId;
                }
            }

            mapLabelsToRender.push_back({name, centerX, centerY, routingNodeId}); 
        } // <-- THIS WAS THE MISSING CLOSING BRACE CAUSING THE ERRORS
        
        std::cout << "   -> Condensed down to " << mapLabelsToRender.size() << " unique centered locations." << std::endl;

        std::cout << "[PIPELINE] Allocating graphical viewport frame..." << std::endl;
        GCRApp app(realGraph, mapLabelsToRender);

        std::cout << "\n=======================================================" << std::endl;
        std::cout << " PIPELINE CONNECTED: Running Runtime UI Loop Context..." << std::endl;
        std::cout << "=======================================================\n" << std::endl;

        app.run();

    } catch (const std::exception& e) {
        std::cerr << "FATAL PIPELINE EXCEPTION: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "SIMT Engine terminated safely." << std::endl;
    return 0;
}