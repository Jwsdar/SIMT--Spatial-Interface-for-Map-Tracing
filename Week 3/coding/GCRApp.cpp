#include <algorithm> // <-- Fixes the std::transform error
#include <cctype>    // <-- Needed for clean ::tolower execution
#include "GCRApp.h"
#include "PathfindingStrategy.h"
#include <chrono>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <limits> 
#include <cmath>
uint64_t GCRApp::findClosestNodeByLocationName(const std::string& inputStr) {
    if (inputStr.empty()) return 0;

    std::string cleanInput = "";
    for (char c : inputStr) {
        if (c == '_') continue; 
        if (static_cast<unsigned char>(c) >= 32) {
            cleanInput += c;
        }
    }

    while (!cleanInput.empty() && std::isspace(static_cast<unsigned char>(cleanInput.back()))) {
        cleanInput.pop_back();
    }
    while (!cleanInput.empty() && std::isspace(static_cast<unsigned char>(cleanInput.front()))) {
        cleanInput.erase(cleanInput.begin());
    }

    if (cleanInput.empty()) return 0;

    std::transform(cleanInput.begin(), cleanInput.end(), cleanInput.begin(), [](unsigned char c) {
        return std::tolower(c);
    });
    
    std::cout << "[GCR Engine] Running Matcher on Clean String: \"" << cleanInput << "\"" << std::endl;

    for (const auto& label : mapLabels) {
        std::string targetLabel = label.text; 
        
        std::transform(targetLabel.begin(), targetLabel.end(), targetLabel.begin(), [](unsigned char c) {
            return std::tolower(c);
        });
        
        if (targetLabel.find(cleanInput) != std::string::npos) {
            std::cout << "[GCR Engine] SUCCESS! Matched with Database Node: \"" 
                      << label.text << "\" -> Base Target ID: " << label.id << std::endl;
            
            // Returns the pre-snapped, fully connected highway node instantly!
            return label.id;
        }
    }
    
    std::cout << "[GCR Engine] ERROR: Unable to locate match for: \"" << cleanInput << "\"" << std::endl;
    return 0; 
}

void GCRApp::processRoutingRequest() {
    hasRoutingError = false;
    hasBenchmarkResults = false;
    errorMessage = "";
    pathGeometry.clear();

    // 1. Resolve typed text strings to Node IDs
    uint64_t resolvedStart = findClosestNodeByLocationName(startInputStr);
    uint64_t resolvedEnd = findClosestNodeByLocationName(endInputStr);

    // Validate if the input text mapped to existing graph coordinates
    if (resolvedStart == 0 || resolvedEnd == 0) {
        hasRoutingError = true;
        if (resolvedStart == 0 && resolvedEnd == 0) errorMessage = "Error: Both locations are not on the map!";
        else if (resolvedStart == 0) errorMessage = "Error: Start location is not on the map!";
        else errorMessage = "Error: End location is not on the map!";
        return;
    }

    // Allocate temporary metrics collectors to receive the strategy references
    size_t dijkstraVerts = 0;
    size_t dijkstraEdges = 0;
    size_t astarVerts = 0;
    size_t astarEdges = 0;

    // 2. Profile Dijkstra's Strategy
    DijkstraStrategy dijkstraSolver;
    auto tStartDijkstra = std::chrono::high_resolution_clock::now();
    std::vector<uint64_t> dijkstraRoute = dijkstraSolver.findPath(
        mapGraph, resolvedStart, resolvedEnd, dijkstraVerts, dijkstraEdges
    );
    auto tEndDijkstra = std::chrono::high_resolution_clock::now();
    
    dijkstraData.latencyMs = std::chrono::duration<double, std::milli>(tEndDijkstra - tStartDijkstra).count();
    dijkstraData.verticesTraversed = dijkstraVerts;
    dijkstraData.edgesTraversed = dijkstraEdges;

    // 3. Profile A* Strategy
    AStarStrategy astarSolver;
    auto tStartAStar = std::chrono::high_resolution_clock::now();
    std::vector<uint64_t> astarRoute = astarSolver.findPath(
        mapGraph, resolvedStart, resolvedEnd, astarVerts, astarEdges
    );
    auto tEndAStar = std::chrono::high_resolution_clock::now();
    
    astarData.latencyMs = std::chrono::duration<double, std::milli>(tEndAStar - tStartAStar).count();
    astarData.verticesTraversed = astarVerts;
    astarData.edgesTraversed = astarEdges;

    // 4. Handle Unreachable Road Connections
    if (astarRoute.empty()) {
        hasRoutingError = true;
        errorMessage = "Error: No legal road path exists between locations!";
        return;
    }

    // 5. Populate the visible geometry path once using clean public accessors
    for (uint64_t nodeId : astarRoute) {
        if (mapGraph.hasNode(nodeId)) {
            const auto& vertex = mapGraph.getVertex(nodeId);
            sf::Vertex pathNode;
            pathNode.position = sf::Vector2f(static_cast<float>(vertex.longitude), 
                                             static_cast<float>(vertex.latitude));
            pathNode.color = sf::Color(255, 69, 0); // High contrast orange
            pathGeometry.append(pathNode);
        }
    }

    // Anchor the floating benchmarks UI tracking box directly on top of the destination node position
    const auto& destVertex = mapGraph.getVertex(resolvedEnd);
    destinationPixelPos = sf::Vector2f(static_cast<float>(destVertex.longitude), 
                                       static_cast<float>(destVertex.latitude));

    // =================================================================
    // 6. TRUE PIXEL DISTANCE (EUCLIDEAN MATH)
    // =================================================================
    const auto& startVertex = mapGraph.getVertex(resolvedStart);
    
    // Calculate the difference between X (longitude) and Y (latitude) planar coordinates
    float dx = static_cast<float>(destVertex.longitude - startVertex.longitude);
    float dy = static_cast<float>(destVertex.latitude - startVertex.latitude);

    // Pythagorean theorem to get the true straight-line pixel distance
    routeDistancePixels = std::sqrt((dx * dx) + (dy * dy));

    // Reset the UI box spawn position flag so it jumps to the new destination
    isMetricsPosInitialized = false; 

    hasBenchmarkResults = true;
}

// 1. Updated Constructor to support map data label vector pipelines
// 2. Fixed sf::VideoMode to use explicit sf::Vector2u to prevent MinGW syntax errors
GCRApp::GCRApp(const SpatialGraph& graph, const std::vector<MapLabel>& rawLabels)
    : window(sf::VideoMode(sf::Vector2u(1280u, 720u)), "SIMT - Ground Control Resource POC"),
      mapGraph(graph),
      mapGeometry(sf::PrimitiveType::Lines),
      pathGeometry(sf::PrimitiveType::LineStrip),
      isDragging(false)
{
    // 1. Load your clean Urdu/Unicode compatible font
    if (!mapFont.openFromFile("C:/Windows/Fonts/tahoma.ttf")) {
        std::cerr << "[WARNING] System typography asset failed to load. Skipping text rendering." << std::endl;
    }

    // --- NEW: Change the live title bar window icon ---
    sf::Image titleBarIcon;
    // Make sure the string matches your exact filename
    if (titleBarIcon.loadFromFile("favicon.png")) { 
        // Extracts the pixel buffer array and applies it to the active window
        window.setIcon(titleBarIcon.getSize(), titleBarIcon.getPixelsPtr());
    } else {
        std::cerr << "[WARNING] Live title-bar icon asset failed to load." << std::endl;
    }

    // 2. Configure your camera views as normal
    cameraView.setSize(sf::Vector2f(1280.f, 720.f));
    cameraView.setCenter(sf::Vector2f(640.f, 360.f));
    window.setView(cameraView);
    

    // Fixed markers Origin settings to use sf::Vector2f
    startMarker.setRadius(5.f);
    startMarker.setFillColor(sf::Color::Green);
    startMarker.setOrigin(sf::Vector2f(5.f, 5.f));

    endMarker.setRadius(5.f);
    endMarker.setFillColor(sf::Color::Red);
    endMarker.setOrigin(sf::Vector2f(5.f, 5.f));

    // Populate lines and text data layers
    buildMapGeometry();
    buildLabels(rawLabels);
}

void GCRApp::buildMapGeometry() {
    mapGeometry.clear();
    mapGeometry.setPrimitiveType(sf::PrimitiveType::Lines);

    const auto& adjMap = mapGraph.getAdjacencyMap();

    for (const auto& [id, vertex] : adjMap) {
        float startX = static_cast<float>(vertex.longitude);
        float startY = static_cast<float>(vertex.latitude);
        
        for (const auto& edge : vertex.adjacencyList) { 
            auto it = adjMap.find(edge.targetNodeId); 
            
            if (it != adjMap.end()) {
                const auto& destVertex = it->second;
                
                float endX = static_cast<float>(destVertex.longitude);
                float endY = static_cast<float>(destVertex.latitude);
                
                // Changed from braced initialization to explicit constructors for MinGW stability
                // New SFML 3 style aggregate struct initialization
                mapGeometry.append(sf::Vertex{sf::Vector2f({startX, startY}), sf::Color(150, 150, 150)});

                mapGeometry.append(sf::Vertex{sf::Vector2f({endX, endY}), sf::Color(150, 150, 150)});
            }
        }
    }
}

void GCRApp::buildLabels(const std::vector<MapLabel>& rawLabels) {
    // 1. Save the raw data structures for your pathfinding/search queries
    mapLabels = rawLabels; 
    
    // 2. Clear and populate your separate visual layout container
    mapLabelDrawables.clear();
    
    for (const auto& item : rawLabels) {
        sf::Text text(mapFont);
        
        // Convert standard string to UTF-8 SFML String safely
        sf::String utf8String = sf::String::fromUtf8(item.text.begin(), item.text.end());
        text.setString(utf8String);
        
        text.setCharacterSize(12);              
        text.setFillColor(sf::Color::Cyan);      
        text.setPosition(sf::Vector2f(item.x + 5.f, item.y - 5.f));
        
        // Push to the visual rendering vector instead
        mapLabelDrawables.push_back(text);
    }
    std::cout << "Data Check: Loaded " << mapLabels.size() << " database structs into memory." << std::endl;
}

void GCRApp::processEvents() {
    while (const std::optional<sf::Event> eventOpt = window.pollEvent()) {
        const sf::Event& event = *eventOpt;

        // ==========================================
        // 1. TEXT KEYBOARD INTERACTION LAYER
        // ==========================================
        // FIXED: Changed event->getIf over to event.getIf to match object reference accessor layout
        if (const auto* textEntered = event.getIf<sf::Event::TextEntered>()) {
            uint32_t unicodeValue = textEntered->unicode;

            // Clear old error messages as soon as the user starts correcting their text entry
            hasRoutingError = false; 

            if (unicodeValue == 13) { // Enter Key pressed
                processRoutingRequest();
            }
            else if (unicodeValue == 9) { // Tab Key pressed
                isStartActive = !isStartActive; // Toggle active text buffer focus
            }
            else if (unicodeValue == 8) { // Backspace Key handling
                if (isStartActive && !startInputStr.empty()) startInputStr.pop_back();
                else if (!isStartActive && !endInputStr.empty()) endInputStr.pop_back();
            }
            else if (unicodeValue >= 32) { 
                // Append native wide Unicode text layouts safely
                if (isStartActive) {
                    sf::Utf32::toUtf8(&unicodeValue, &unicodeValue + 1, std::back_inserter(startInputStr));
                } else {
                    sf::Utf32::toUtf8(&unicodeValue, &unicodeValue + 1, std::back_inserter(endInputStr));
                }
            }
        }

        // ==========================================
        // 2. WINDOW SYSTEM CLOSING CONTROLS (Isolated block)
        // ==========================================
        if (event.is<sf::Event::Closed>()) {
            window.close();
        }
        
        if (const auto* keyPressed = event.getIf<sf::Event::KeyPressed>()) {
            if (keyPressed->scancode == sf::Keyboard::Scan::Escape) {
                window.close();
            }
        }

        // ==========================================
        // 3. MOUSE CAMERA & MAP VIEW INTERACTION (Completely unlinked from key states)
        // ==========================================
        if (const auto* mouseWheel = event.getIf<sf::Event::MouseWheelScrolled>()) {
            if (mouseWheel->delta > 0) {
                cameraView.zoom(0.9f);
            } else if (mouseWheel->delta < 0) {
                cameraView.zoom(1.1f);
            }
            window.setView(cameraView);
        }
        // Inside your loop: while (window.pollEvent(event)) OR while (auto event = window.pollEvent())

        // --- 1. HANDLING MOUSE BUTTON PRESSES (Using dot notation) ---
        if (const auto* mouseButtonPressed = event.getIf<sf::Event::MouseButtonPressed>()) {
            if (mouseButtonPressed->button == sf::Mouse::Button::Left) {
                // Enforce top-bar input HUD protection so dragging doesn't trigger when clicking inputs
                if (mouseButtonPressed->position.y > 40) { 
                    isDragging = true;
                    // Record where the drag initiated using your existing oldMousePos variable
                    oldMousePos = window.mapPixelToCoords(mouseButtonPressed->position, cameraView);
                }
            }
        }

        // --- 2. HANDLING MOUSE BUTTON RELEASES (Using dot notation) ---
        if (const auto* mouseButtonReleased = event.getIf<sf::Event::MouseButtonReleased>()) {
            if (mouseButtonReleased->button == sf::Mouse::Button::Left) {
                isDragging = false;
            }
        }

        // --- 3. HANDLING MOUSE DRAG/MOVEMENT (Using dot notation) ---
        if (const auto* mouseMoved = event.getIf<sf::Event::MouseMoved>()) {
            if (isDragging) {
                // Map the current mouse pixel layout straight into the active coordinate viewport
                sf::Vector2f currentMouseWorldPos = window.mapPixelToCoords(mouseMoved->position, cameraView);
                
                // Calculate the difference between the anchor point and current frame position
                sf::Vector2f delta = oldMousePos - currentMouseWorldPos;
                
                // Shift camera view position smoothly
                cameraView.move(delta);
                
                // Re-anchor oldMousePos using the newly shifted view context to prevent drift spikes
                oldMousePos = window.mapPixelToCoords(mouseMoved->position, cameraView);
            }
        }
    }
}

void GCRApp::update() {
    // Update logic frames here if needed
}

void GCRApp::render() {
    // 1. Wipe out the previous frame completely
    window.clear(sf::Color::Black);

    // =========================================================================
    // LAYER GROUP A: CAMERA VIEW BOUND CONTEXT (Moves/zooms when user drags map)
    // =========================================================================
    window.setView(cameraView);

    // Render Layer 1: Base road network lines
    window.draw(mapGeometry);

    // Render Layer 2: Pathfinding route overlay (Drawn directly over streets)
    if (pathGeometry.getVertexCount() > 0) {
        window.draw(pathGeometry);
    }

    // Render Layer 3: Spatial text labels (City coordinates and Urdu names)
    // FIXED: Iterating over mapLabelDrawables (sf::Text layouts) instead of mapLabels (data structs)
    for (const auto& labelText : mapLabelDrawables) {
        window.draw(labelText);
    }

    // Render Layer 4: Floating Telemetry Comparison Box (Pinned over destination node)
    if (hasBenchmarkResults) {
        // 1. Initialize the draggable box position near the destination coordinates on the first run
        if (!isMetricsPosInitialized) {
            // Shifted slightly higher (140.f) to account for the taller box
            metricsBoxPos = sf::Vector2f(destinationPixelPos.x - 275.f, destinationPixelPos.y - 140.f);
            isMetricsPosInitialized = true;
        }

        // 2. Set up a translucent dark background box (Increased height to 125.f for the 3rd text line)
        sf::RectangleShape floatBox(sf::Vector2f(550.f, 125.f)); 
        floatBox.setFillColor(sf::Color(20, 20, 20, 230)); 
        floatBox.setOutlineColor(sf::Color(0, 255, 255)); 
        floatBox.setOutlineThickness(1.f);
        
        // Position the box using the draggable state coordinate instead of a fixed anchor
        floatBox.setPosition(metricsBoxPos);

        // 3. Calibration scale conversion multiplier (Adjust the 1.5f value to match your map's real-world meters scale)
        float estimatedMeters = routeDistancePixels * 7.14f;

        // 4. Stitch the profiled algorithm benchmark calculations AND the distance metrics together
        std::stringstream ss;
        ss << "Dijkstra: " << dijkstraData.latencyMs << " ms (" 
           << dijkstraData.verticesTraversed << " nodes settled, " 
           << dijkstraData.edgesTraversed << " edges checked)\n"
           << "A* Engine: " << astarData.latencyMs << " ms (" 
           << astarData.verticesTraversed << " nodes settled, " 
           << astarData.edgesTraversed << " edges checked)\n"
           << "Route Distance: " << std::fixed << std::setprecision(1) << estimatedMeters << " meters (" 
           << static_cast<int>(routeDistancePixels) << " px straight-line)"; 

        // --- SFML 3.x COMPLIANT RENDER FIX ---
        // In SFML 3.x, we pass the font directly into the constructor or set it first
        sf::Text renderText(mapFont); 
        
        // Assign the string explicitly AFTER the font is linked
        renderText.setString(ss.str());     
        
        // SFML 3.x formatting rules
        renderText.setCharacterSize(14);    
        renderText.setFillColor(sf::Color::White);
        
        // Position the text safely inside the floating box margins
        renderText.setPosition(sf::Vector2f(floatBox.getPosition().x + 15.f, floatBox.getPosition().y + 15.f));

        // Draw both components safely onto your SFML 3.x window context
        window.draw(floatBox);
        window.draw(renderText);
    }

    // =========================================================================
    // LAYER GROUP B: ABSOLUTE SCREEN FRAME CONTEXT (Locked overlay HUD)
    // =========================================================================
    // Swapping back to the default view forces coordinates to align directly to the monitor
    window.setView(window.getDefaultView());

    // Render Layer 5: Persistent console text entry header panel
    sf::RectangleShape topBarPanel(sf::Vector2f(1280.f, 40.f));
    topBarPanel.setFillColor(sf::Color(30, 30, 30));
    topBarPanel.setPosition(sf::Vector2f(0.f, 0.f));
    window.draw(topBarPanel);

    // Render Layer 6: Dynamic Text Input Field overlays
    // Appends a blinking underscore symbol cursor depending on which field is currently active
    std::string uiInputDisplay = "START: " + startInputStr + (isStartActive ? "_" : "") + 
                                 "   |   END: " + endInputStr + (!isStartActive ? "_" : "") +
                                 "   [TAB: Switch Fields | ENTER: Execute Route Search]";
    
    sf::Text uiText(mapFont);
    uiText.setString(sf::String::fromUtf8(uiInputDisplay.begin(), uiInputDisplay.end()));
    uiText.setCharacterSize(13);
    uiText.setFillColor(sf::Color::Cyan);
    uiText.setPosition(sf::Vector2f(15.f, 12.f));
    window.draw(uiText);

    // Render Layer 7: Error Alert Banner (Utmost foreground stencil overlay)
    // Pops up dynamically over the screen if a string lookup query validation fails
    if (hasRoutingError) {
        sf::RectangleShape errorBox(sf::Vector2f(450.f, 40.f));
        errorBox.setFillColor(sf::Color(180, 0, 0)); // Warning Red
        errorBox.setPosition(sf::Vector2f(415.f, 60.f));
        
        sf::Text errTxt(mapFont);
        errTxt.setString(sf::String::fromUtf8(errorMessage.begin(), errorMessage.end()));
        errTxt.setCharacterSize(13);
        errTxt.setFillColor(sf::Color::White);
        errTxt.setPosition(sf::Vector2f(430.f, 72.f));
        
        window.draw(errorBox);
        window.draw(errTxt);
    }

    // 2. Flip the graphics back-buffer out to display the compiled layers cleanly onto the monitor
    window.display();
}

void GCRApp::run() {
    while (window.isOpen()) {
        processEvents();
        update();
        render();
    }
}

// Implementation of the IObserver interface callback
void GCRApp::onPathCalculated(const std::vector<uint64_t>& path, 
                              double executionTimeMs, 
                              double totalDistance,
                              std::string algorithmUsed) 
{
    // Implementation pending
}

// Stub implementation for the screenToLatLon helper
sf::Vector2f GCRApp::screenToLatLon(float screenX, float screenY) {
    return sf::Vector2f(0.f, 0.f); 
}