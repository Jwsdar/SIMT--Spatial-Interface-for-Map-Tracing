#include "MapAdapter.h"
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <cmath>

// Constructor
OSMXmlAdapter::OSMXmlAdapter() : isLoaded(false) {}

// Load the physical file into memory using TinyXML2
bool OSMXmlAdapter::loadMapFile(const std::string& filepath) {
    tinyxml2::XMLError result = xmlDoc.LoadFile(filepath.c_str());
    if (result == tinyxml2::XML_SUCCESS) {
        isLoaded = true;
        return true;
    }
    std::cerr << "Failed to load OSM file. Error code: " << result << std::endl;
    return false;
}

// Pass 1 & Pass 2: Extract nodes and propagate area/campus names (like NUTECH)
std::vector<Node> OSMXmlAdapter::extractNodes() {
    std::vector<Node> nodes;
    if (!isLoaded) return nodes;

    tinyxml2::XMLElement* root = xmlDoc.FirstChildElement("osm");
    if (!root) return nodes;

    // Map to keep track of where each Node ID lives in our 'nodes' vector (O(1) lookups)
    std::unordered_map<uint64_t, size_t> nodeIndexMap;

    // --- PASS 1: Extract Base Nodes ---
    for (tinyxml2::XMLElement* nodeElem = root->FirstChildElement("node"); 
         nodeElem != nullptr; 
         nodeElem = nodeElem->NextSiblingElement("node")) {
        
        Node n;
        nodeElem->QueryUnsigned64Attribute("id", &n.id);
        nodeElem->QueryDoubleAttribute("lat", &n.latitude);
        nodeElem->QueryDoubleAttribute("lon", &n.longitude);
        n.name = ""; 

        // Check for direct point-of-interest name tags
        for (tinyxml2::XMLElement* tagElem = nodeElem->FirstChildElement("tag");
             tagElem != nullptr;
             tagElem = tagElem->NextSiblingElement("tag")) {
            
            const char* keyAttr = tagElem->Attribute("k");
            if (keyAttr && std::string(keyAttr) == "name") {
                const char* valAttr = tagElem->Attribute("v");
                if (valAttr) {
                    n.name = std::string(valAttr); 
                }
                break; 
            }
        }
        
        nodeIndexMap[n.id] = nodes.size();
        nodes.push_back(n);
    }

    // --- PASS 2: Broadcast Boundary Names (e.g., Campus Polygons) ---
    for (tinyxml2::XMLElement* wayElem = root->FirstChildElement("way"); 
         wayElem != nullptr; 
         wayElem = wayElem->NextSiblingElement("way")) {
        
        std::string wayName = "";
        
        for (tinyxml2::XMLElement* tagElem = wayElem->FirstChildElement("tag");
             tagElem != nullptr;
             tagElem = tagElem->NextSiblingElement("tag")) {
            
            const char* kAttr = tagElem->Attribute("k");
            if (kAttr && std::string(kAttr) == "name") {
                const char* vAttr = tagElem->Attribute("v");
                if (vAttr) {
                    wayName = std::string(vAttr);
                }
                break;
            }
        }

        // If this area has a name, update all its structural member nodes
        if (!wayName.empty()) {
            for (tinyxml2::XMLElement* ndElem = wayElem->FirstChildElement("nd");
                 ndElem != nullptr;
                 ndElem = ndElem->NextSiblingElement("nd")) {
                
                uint64_t refId = 0;
                ndElem->QueryUnsigned64Attribute("ref", &refId);
                
                auto it = nodeIndexMap.find(refId);
                if (it != nodeIndexMap.end()) {
                    nodes[it->second].name = wayName; 
                }
            }
        }
    }
    
    return nodes;
}

// Extract roads for the Core Spatial Routing Graph (WBS 1.2.1)
std::vector<Way> OSMXmlAdapter::extractWays() {
    std::vector<Way> ways;
    if (!isLoaded) return ways;

    tinyxml2::XMLElement* root = xmlDoc.FirstChildElement("osm");
    if (!root) return ways;

    for (tinyxml2::XMLElement* wayElem = root->FirstChildElement("way"); 
         wayElem != nullptr; 
         wayElem = wayElem->NextSiblingElement("way")) {
        
        bool isRoad = false;

        for (tinyxml2::XMLElement* tagElem = wayElem->FirstChildElement("tag");
             tagElem != nullptr;
             tagElem = tagElem->NextSiblingElement("tag")) {
            
            const char* kAttr = tagElem->Attribute("k");
            if (kAttr && std::string(kAttr) == "highway") {
                isRoad = true;
                break; // Found road indicator tag
            }
        }

        if (isRoad) {
            Way w;
            wayElem->QueryUnsigned64Attribute("id", &w.id);

            for (tinyxml2::XMLElement* ndElem = wayElem->FirstChildElement("nd");
                 ndElem != nullptr;
                 ndElem = ndElem->NextSiblingElement("nd")) {
                
                uint64_t refId = 0;
                ndElem->QueryUnsigned64Attribute("ref", &refId);
                w.node_refs.push_back(refId);
            }
            ways.push_back(w);
        }
    }
    return ways;
}

#include "MapAdapter.h"
#include <iostream>
#include <algorithm>
#include <unordered_map>

// ... (Keep your existing constructor, loadMapFile, extractNodes, and extractWays here)

// Added: Function to display specific nodes by their 64-bit ID
void OSMXmlAdapter::displayNodeDetails(const std::vector<Node>& nodes, uint64_t targetId) {
    // Search the vector for the matching unique ID
    auto it = std::find_if(nodes.begin(), nodes.end(), [targetId](const Node& n) {
        return n.id == targetId;
    });

    std::cout << "\n======================================\n";
    std::cout << "        SIMT NODE INSPECTOR           \n";
    std::cout << "======================================\n";

    if (it != nodes.end()) {
        std::cout << "Status    : FOUND\n";
        std::cout << "Node ID   : " << it->id << "\n";
        std::cout << "Latitude  : " << it->latitude << "\n";
        std::cout << "Longitude : " << it->longitude << "\n";
        
        // Print the name or point out if it's an anonymous segment point
        if (it->name.empty()) {
            std::cout << "Location  : [No name tag assigned (Anonymous Routing Vertex)]\n";
        } else {
            std::cout << "Location  : " << it->name << "\n";
        }
    } else {
        std::cout << "Status    : NOT FOUND\n";
        std::cout << "Reason    : Node ID " << targetId << " is outside your map's bounding box.\n";
    }
    std::cout << "======================================\n\n";
}


// WBS 1.1.2: Parse Explicit Bounding Box from <bounds> tag
bool OSMXmlAdapter::extractBounds() {
    if (!isLoaded) return false;

    tinyxml2::XMLElement* root = xmlDoc.FirstChildElement("osm");
    if (!root) return false;

    tinyxml2::XMLElement* boundsElem = root->FirstChildElement("bounds");
    if (boundsElem) {
        boundsElem->QueryDoubleAttribute("minlat", &mapBounds.minLat);
        boundsElem->QueryDoubleAttribute("maxlat", &mapBounds.maxLat);
        boundsElem->QueryDoubleAttribute("minlon", &mapBounds.minLon);
        boundsElem->QueryDoubleAttribute("maxlon", &mapBounds.maxLon);
        return true;
    }

    // Fallback default bounds for NUTECH/Islamabad segment clip if <bounds> tag missing
    std::cerr << "Warning: <bounds> tag not found. Using fallback geometry.\n";
    mapBounds = { 33.6300, 33.6600, 72.9800, 73.0300 };
    return false;
}

// WBS 1.1.2: Linear Bounding Box Normalization & Coordinate Conversion Engine
ScreenPoint OSMXmlAdapter::projectToScreen(double lat, double lon, float windowWidth, float windowHeight) {
    ScreenPoint point;

    // 1. Calculate relative percentage location within geographic box bounds (0.0 to 1.0)
    double normX = (lon - mapBounds.minLon) / (mapBounds.maxLon - mapBounds.minLon);
    double normY = (lat - mapBounds.minLat) / (mapBounds.maxLat - mapBounds.minLat);

    // 2. Map to coordinate pixels matching window parameters.
    // CRITICAL: In computer graphics (SFML), Y increases downwards! 
    // To keep the North side at the top of your monitor, invert the Y mapping value.
    point.x = static_cast<float>(normX * windowWidth);
    point.y = static_cast<float>((1.0 - normY) * windowHeight);

    return point;
}