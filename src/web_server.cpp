/**
 * @file web_server.cpp
 * @brief Web server core implementation
 * 
 * This file contains the core WebServer class implementation:
 * - Server initialization and lifecycle
 * - Route setup (wiring handlers to endpoints)
 * - WebSocket handling
 * - Helper methods
 * - Basic status/control handlers
 * 
 * Handler implementations are split across multiple files for maintainability:
 * - web_handlers_tracking.cpp: Satellite tracking, TLE management, passes
 * - web_handlers_system.cpp: Health monitoring, SD card, UDP broadcast
 */

#include "web_server.h"
#include "stepper_control.h"
#include "tracking_engine.h"
#include "tle_manager.h"
#include "wifi_manager.h"
#include "gps.h"

#include "nvs_storage.h"
#include "rotctld_server.h"
#include "health_monitor.h"
#include "sd_storage.h"
#include "external_api.h"
#include "udp_broadcast.h"
#include <SPIFFS.h>

// Global instance
WebServer webServer;

// =============================================================================
// CONSTRUCTOR / DESTRUCTOR
// =============================================================================

WebServer::WebServer()
    : _server(nullptr)
    , _ws(nullptr)
    , _running(false)
    , _spiffsAvailable(false)
    , _lastBroadcast(0)
{
}

WebServer::~WebServer() {
    stop();
}

// =============================================================================
// LIFECYCLE METHODS
// =============================================================================

bool WebServer::begin() {
    DEBUG_PRINTLN("WebServer: Initializing...");
    
    // Ensure WiFi is ready before starting TCP services
    // This prevents "tcp_alloc: Required to lock TCPIP core" assertion failures
    if (WiFi.getMode() == WIFI_OFF) {
        DEBUG_PRINTLN("WebServer: ERROR - WiFi not initialized!");
        return false;
    }
    
    // Yield to let TCP/IP stack tasks run
    for (int i = 0; i < 10; i++) {
        delay(50);
        yield();
    }
    
    // Initialize SPIFFS for static files (optional - API works without it)
    _spiffsAvailable = SPIFFS.begin(true);
    if (!_spiffsAvailable) {
        DEBUG_PRINTLN("WebServer: SPIFFS mount failed - using built-in pages");
        DEBUG_PRINTLN("WebServer: Run 'pio run -t uploadfs' to upload web files");
    }
    
    // Create server
    _server = new AsyncWebServer(WEB_SERVER_PORT);
    _ws = new AsyncWebSocket("/ws");
    
    // Setup routes
    setupRoutes();
    setupAPIRoutes();
    setupAdvancedRoutes();
    setupWebSocketHandler();
    
    // Start server
    _server->begin();
    _running = true;
    
    DEBUG_PRINTF("WebServer: Started on port %d\n", WEB_SERVER_PORT);
    return true;
}

void WebServer::stop() {
    if (_running) {
        if (_server) {
            _server->end();
        }
        
        // Delete all JSON handlers to prevent memory leak
        for (auto* handler : _jsonHandlers) {
            if (handler) {
                delete handler;
            }
        }
        _jsonHandlers.clear();
        
        if (_ws) {
            delete _ws;
            _ws = nullptr;
        }
        
        if (_server) {
            delete _server;
            _server = nullptr;
        }
        
        _running = false;
        DEBUG_PRINTLN("WebServer: Stopped");
    }
}

void WebServer::update() {
    if (!_running) return;
    
    // Cleanup disconnected WebSocket clients
    _ws->cleanupClients();
    
    // Periodic status broadcast
    uint32_t now = millis();
    if (now - _lastBroadcast >= STATUS_UPDATE_INTERVAL_MS) {
        broadcastStatus();
        _lastBroadcast = now;
    }
}

// =============================================================================
// WEBSOCKET METHODS
// =============================================================================

void WebServer::broadcastStatus() {
    // Build JSON first, then send atomically
    // textAll() handles empty client list gracefully, so no count check needed
    const char* json = buildStatusJSON();
    _ws->textAll(json);
}

void WebServer::sendEvent(const char* event, const char* data) {
    // Build complete message before sending to avoid race conditions
    StaticJsonDocument<JSON_SIZE_SMALL> doc;
    doc["event"] = event;
    doc["data"] = data;
    
    String json;
    serializeJson(doc, json);
    
    // textAll() handles empty client list gracefully
    _ws->textAll(json);
}

int WebServer::getClientCount() const {
    return _ws ? _ws->count() : 0;
}

void WebServer::setupWebSocketHandler() {
    _ws->onEvent([this](AsyncWebSocket* server, AsyncWebSocketClient* client,
                        AwsEventType type, void* arg, uint8_t* data, size_t len) {
        onWebSocketEvent(server, client, type, arg, data, len);
    });
    
    _server->addHandler(_ws);
}

void WebServer::onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                                  AwsEventType type, void* arg, uint8_t* data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            DEBUG_PRINTF("WebSocket client connected: %u\n", client->id());
            // Send initial status
            client->text(buildStatusJSON());
            break;
            
        case WS_EVT_DISCONNECT:
            DEBUG_PRINTF("WebSocket client disconnected: %u\n", client->id());
            break;
            
        case WS_EVT_DATA: {
            // Handle incoming WebSocket messages
            AwsFrameInfo* info = (AwsFrameInfo*)arg;
            if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
                data[len] = 0;
                DEBUG_PRINTF("WebSocket message: %s\n", (char*)data);
                
                // Parse JSON command
                StaticJsonDocument<2048> doc;
                DeserializationError error = deserializeJson(doc, (char*)data);
                if (!error) {
                    String cmd = doc["cmd"] | "";
                    // Handle WebSocket commands if needed
                }
            }
            break;
        }
            
        default:
            break;
    }
}

// =============================================================================
// ROUTE SETUP - STATIC FILES
// =============================================================================

void WebServer::setupRoutes() {
    // Serve static files from SPIFFS only if available
    if (_spiffsAvailable) {
        _server->serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
    }
    
    // Root handler (for SPA routing)
    _server->on("/", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleRoot(request);
    });
    
    // 404 handler
    _server->onNotFound([this](AsyncWebServerRequest* request) {
        handleNotFound(request);
    });
}

// =============================================================================
// ROUTE SETUP - BASIC API
// =============================================================================

void WebServer::setupAPIRoutes() {
    // Status endpoints
    _server->on("/api/status", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetStatus(request);
    });
    
    _server->on("/api/position", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetPosition(request);
    });
    
    _server->on("/api/gps", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetGPS(request);
    });
    
    _server->on("/api/config", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetConfig(request);
    });
    
    // Control endpoints
    AsyncCallbackJsonWebHandler* posHandler = new AsyncCallbackJsonWebHandler("/api/position",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            handleSetPosition(request, json);
        });
    _server->addHandler(posHandler);
    _jsonHandlers.push_back(posHandler);
    
    _server->on("/api/stop", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleStop(request);
    });
    
    _server->on("/api/park", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handlePark(request);
    });
    
    _server->on("/api/home", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleHome(request);
    });
    
    _server->on("/api/sethome", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleSetHome(request);
    });
    
    AsyncCallbackJsonWebHandler* calibHandler = new AsyncCallbackJsonWebHandler("/api/calibrate",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            handleCalibrate(request, json);
        });
    _server->addHandler(calibHandler);
    _jsonHandlers.push_back(calibHandler);
    
    // Tracking endpoints (handlers in web_handlers_tracking.cpp)
    AsyncCallbackJsonWebHandler* trackHandler = new AsyncCallbackJsonWebHandler("/api/track",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            handleStartTracking(request, json);
        });
    _server->addHandler(trackHandler);
    _jsonHandlers.push_back(trackHandler);
    
    _server->on("/api/track", HTTP_DELETE, [this](AsyncWebServerRequest* request) {
        handleStopTracking(request);
    });
    
    _server->on("/api/nextpass", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetNextPass(request);
    });
    
    // TLE endpoints (handlers in web_handlers_tracking.cpp)
    _server->on("/api/tle", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetTLEs(request);
    });
    
    AsyncCallbackJsonWebHandler* tleHandler = new AsyncCallbackJsonWebHandler("/api/tle",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            handleLoadTLE(request, json);
        });
    _server->addHandler(tleHandler);
    _jsonHandlers.push_back(tleHandler);
    
    AsyncCallbackJsonWebHandler* fetchHandler = new AsyncCallbackJsonWebHandler("/api/tle/fetch",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            handleFetchTLE(request, json);
        });
    _server->addHandler(fetchHandler);
    _jsonHandlers.push_back(fetchHandler);
    
    // WiFi endpoints
    _server->on("/api/wifi/networks", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetNetworks(request);
    });
    
    AsyncCallbackJsonWebHandler* wifiHandler = new AsyncCallbackJsonWebHandler("/api/wifi/connect",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            handleConnectWiFi(request, json);
        });
    _server->addHandler(wifiHandler);
    _jsonHandlers.push_back(wifiHandler);
    
    _server->on("/api/wifi/status", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetWiFiStatus(request);
    });
    
    // System endpoints
    _server->on("/api/system/info", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetSystemInfo(request);
    });
    
    _server->on("/api/system/reboot", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleReboot(request);
    });
    
    _server->on("/api/system/reset", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleFactoryReset(request);
    });
    
    // GPS manual position endpoint
    AsyncCallbackJsonWebHandler* gpsManualHandler = new AsyncCallbackJsonWebHandler("/api/gps/manual",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            handleSetManualGPS(request, json);
        });
    _server->addHandler(gpsManualHandler);
    _jsonHandlers.push_back(gpsManualHandler);
}

// =============================================================================
// ROUTE SETUP - ADVANCED (handlers in web_handlers_*.cpp files)
// =============================================================================

void WebServer::setupAdvancedRoutes() {
    // Multi-pass prediction (handlers in web_handlers_tracking.cpp)
    _server->on("/api/passes", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetPasses(request);
    });
    
    _server->on("/api/schedule", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetSchedule(request);
    });
    
    _server->on("/api/schedule/clear", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleClearSchedule(request);
    });
    
    AsyncCallbackJsonWebHandler* scheduleHandler = new AsyncCallbackJsonWebHandler("/api/schedule",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            handleSchedulePass(request, json);
        });
    _server->addHandler(scheduleHandler);
    _jsonHandlers.push_back(scheduleHandler);
    
    // Doppler (handler in web_handlers_tracking.cpp)
    _server->on("/api/doppler", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetDoppler(request);
    });
    
    // Health monitoring (handlers in web_handlers_system.cpp)
    _server->on("/api/health", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetHealth(request);
    });
    
    _server->on("/api/health/events", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetHealthEvents(request);
    });
    
    _server->on("/api/health/events/clear", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleClearHealthEvents(request);
    });
    
    // SD Card (handlers in web_handlers_system.cpp)
    _server->on("/api/sd/status", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleSDStatus(request);
    });
    
    _server->on("/api/sd/tle", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleSDListTLEs(request);
    });
    
    AsyncCallbackJsonWebHandler* sdImportHandler = new AsyncCallbackJsonWebHandler("/api/sd/tle/import",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            handleSDImportTLE(request, json);
        });
    _server->addHandler(sdImportHandler);
    _jsonHandlers.push_back(sdImportHandler);
    
    _server->on("/api/sd/tle/export", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleSDExportTLEs(request);
    });
    
    _server->on("/api/sd/backup", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleSDBackupConfig(request);
    });
    
    _server->on("/api/sd/restore", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleSDRestoreConfig(request);
    });
    
    _server->on("/api/passlog", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetPassLog(request);
    });
    
    _server->on("/api/passlog/export", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleExportPassLog(request);
    });
    
    // External APIs (handlers in web_handlers_tracking.cpp)
    AsyncCallbackJsonWebHandler* n2yoKeyHandler = new AsyncCallbackJsonWebHandler("/api/n2yo/key",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            handleSetN2YOKey(request, json);
        });
    _server->addHandler(n2yoKeyHandler);
    _jsonHandlers.push_back(n2yoKeyHandler);
    
    AsyncCallbackJsonWebHandler* n2yoFetchHandler = new AsyncCallbackJsonWebHandler("/api/n2yo/tle",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            handleFetchN2YOTLE(request, json);
        });
    _server->addHandler(n2yoFetchHandler);
    _jsonHandlers.push_back(n2yoFetchHandler);
    
    _server->on("/api/n2yo/passes", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetVisualPasses(request);
    });
    
    _server->on("/api/satnogs/search", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleSearchSatNOGS(request);
    });
    
    _server->on("/api/satnogs/transmitters", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetTransmitters(request);
    });
    
    AsyncCallbackJsonWebHandler* tleGroupHandler = new AsyncCallbackJsonWebHandler("/api/celestrak/group",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            handleFetchTLEGroup(request, json);
        });
    _server->addHandler(tleGroupHandler);
    _jsonHandlers.push_back(tleGroupHandler);
    
    // UDP Broadcast (handlers in web_handlers_system.cpp)
    _server->on("/api/udp/status", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleUDPStatus(request);
    });
    
    AsyncCallbackJsonWebHandler* udpHandler = new AsyncCallbackJsonWebHandler("/api/udp/config",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            handleUDPConfig(request, json);
        });
    _server->addHandler(udpHandler);
    _jsonHandlers.push_back(udpHandler);
    
    // EME Mode (handlers in web_handlers_tracking.cpp)
    _server->on("/api/eme/enable", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleEnableEME(request);
    });
    
    _server->on("/api/eme/disable", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleDisableEME(request);
    });
    
    _server->on("/api/moon", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetMoonInfo(request);
    });
    
    _server->on("/api/eme/pathloss", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetEMEPathLoss(request);
    });
    
    // Geo Satellite Mode (handlers in web_handlers_tracking.cpp)
    AsyncCallbackJsonWebHandler* geoHandler = new AsyncCallbackJsonWebHandler("/api/geo/point",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            handlePointToGeo(request, json);
        });
    _server->addHandler(geoHandler);
    _jsonHandlers.push_back(geoHandler);
    
    AsyncCallbackJsonWebHandler* geoTrackHandler = new AsyncCallbackJsonWebHandler("/api/geo/track",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            handleEnableGeoTracking(request, json);
        });
    _server->addHandler(geoTrackHandler);
    _jsonHandlers.push_back(geoTrackHandler);
    
    _server->on("/api/geo/disable", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleDisableGeoTracking(request);
    });
    
    // Antenna Pattern (handlers in web_handlers_tracking.cpp)
    _server->on("/api/antenna", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetAntennaPattern(request);
    });
    
    AsyncCallbackJsonWebHandler* antennaHandler = new AsyncCallbackJsonWebHandler("/api/antenna",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            handleSetAntennaPattern(request, json);
        });
    _server->addHandler(antennaHandler);
    _jsonHandlers.push_back(antennaHandler);
    
    // Pre-positioning (handler in web_handlers_tracking.cpp)
    AsyncCallbackJsonWebHandler* preposHandler = new AsyncCallbackJsonWebHandler("/api/preposition",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            handleSetPrePosition(request, json);
        });
    _server->addHandler(preposHandler);
    _jsonHandlers.push_back(preposHandler);
    
    // TLE delete endpoint
    AsyncCallbackJsonWebHandler* tleDeleteHandler = new AsyncCallbackJsonWebHandler("/api/tle/delete",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            handleDeleteTLE(request, json);
        });
    _server->addHandler(tleDeleteHandler);
    _jsonHandlers.push_back(tleDeleteHandler);
    
    // Cancel pass endpoint
    AsyncCallbackJsonWebHandler* cancelPassHandler = new AsyncCallbackJsonWebHandler("/api/schedule/cancel",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            handleCancelPass(request, json);
        });
    _server->addHandler(cancelPassHandler);
    _jsonHandlers.push_back(cancelPassHandler);
}

// =============================================================================
// STATIC FILE HANDLERS
// =============================================================================

// Built-in fallback page when SPIFFS is not available
static const char BUILTIN_INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Satellite Tracker</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #1a1a2e; color: #eee; }
        h1 { color: #4CAF50; }
        .card { background: #16213e; padding: 20px; border-radius: 8px; margin: 10px 0; }
        .status { color: #ffc107; }
        a { color: #4CAF50; }
        code { background: #0f3460; padding: 2px 6px; border-radius: 4px; }
    </style>
</head>
<body>
    <h1>📡 Satellite Tracker</h1>
    <div class="card">
        <h2>⚠️ Web UI Not Uploaded</h2>
        <p>The SPIFFS filesystem is not available. To upload the web interface:</p>
        <ol>
            <li>Run: <code>pio run -t uploadfs</code></li>
            <li>Or use the API directly at <a href="/api/status">/api/status</a></li>
        </ol>
    </div>
    <div class="card">
        <h2>API Endpoints</h2>
        <ul>
            <li><a href="/api/status">GET /api/status</a> - Current status</li>
            <li><a href="/api/position">GET /api/position</a> - Position info</li>
            <li><a href="/api/tle/list">GET /api/tle/list</a> - TLE list</li>
            <li><a href="/api/health">GET /api/health</a> - Health status</li>
        </ul>
    </div>
</body>
</html>
)rawliteral";

void WebServer::handleRoot(AsyncWebServerRequest* request) {
    if (_spiffsAvailable) {
        request->send(SPIFFS, "/index.html", "text/html");
    } else {
        request->send_P(200, "text/html", BUILTIN_INDEX_HTML);
    }
}

void WebServer::handleNotFound(AsyncWebServerRequest* request) {
    // For API routes, return JSON error
    if (request->url().startsWith("/api/")) {
        sendError(request, "Endpoint not found", 404);
    } else {
        // For other routes, serve index.html (SPA routing)
        if (_spiffsAvailable) {
            request->send(SPIFFS, "/index.html", "text/html");
        } else {
            request->send_P(200, "text/html", BUILTIN_INDEX_HTML);
        }
    }
}

// =============================================================================
// BASIC STATUS HANDLERS
// =============================================================================

void WebServer::handleGetStatus(AsyncWebServerRequest* request) {
    StaticJsonDocument<JSON_SIZE_LARGE> doc;
    
    // Position
    doc["azimuth"] = stepperControl.getAzimuth();
    doc["elevation"] = stepperControl.getElevation();
    doc["targetAz"] = stepperControl.getTargetAzimuth();
    doc["targetEl"] = stepperControl.getTargetElevation();
    doc["moving"] = stepperControl.isMoving();
    doc["enabled"] = stepperControl.isEnabled();
    
    // Tracking
    doc["mode"] = (int)trackingEngine.getMode();
    doc["satellite"] = trackingEngine.getSatelliteName();
    doc["satVisible"] = trackingEngine.isSatelliteVisible();
    
    // Satellite position
    if (trackingEngine.hasTLE()) {
        SatellitePosition satPos = trackingEngine.getPosition();
        doc["satAz"] = satPos.azimuth;
        doc["satEl"] = satPos.elevation;
        doc["satRange"] = satPos.range;
    }
    
    // GPS
    doc["gpsValid"] = gps.hasFix();
    doc["gpsSats"] = gps.getSatellites();
    
    // Connection
    doc["wifiConnected"] = wifiManager.isConnected();
    doc["rotctldClients"] = rotctldServer.getClientCount();
    
    sendJSON(request, doc);
}

void WebServer::handleGetPosition(AsyncWebServerRequest* request) {
    StaticJsonDocument<JSON_SIZE_SMALL> doc;
    doc["azimuth"] = stepperControl.getAzimuth();
    doc["elevation"] = stepperControl.getElevation();
    doc["targetAz"] = stepperControl.getTargetAzimuth();
    doc["targetEl"] = stepperControl.getTargetElevation();
    doc["moving"] = stepperControl.isMoving();
    sendJSON(request, doc);
}

void WebServer::handleGetGPS(AsyncWebServerRequest* request) {
    StaticJsonDocument<JSON_SIZE_MEDIUM> doc;
    GPSData data = gps.getData();
    
    doc["valid"] = data.valid;
    doc["latitude"] = data.latitude;
    doc["longitude"] = data.longitude;
    doc["altitude"] = data.altitude;
    doc["satellites"] = data.satellites;
    doc["hdop"] = data.hdop;
    doc["time"] = data.unixTime;
    doc["manual"] = gps.isUsingManualPosition();
    
    sendJSON(request, doc);
}

void WebServer::handleGetConfig(AsyncWebServerRequest* request) {
    StaticJsonDocument<JSON_SIZE_MEDIUM> doc;
    
    doc["version"] = FIRMWARE_VERSION;
    doc["model"] = DEVICE_MODEL;
    doc["azMin"] = AZ_MIN_DEG;
    doc["azMax"] = AZ_MAX_DEG;
    doc["elMin"] = EL_MIN_DEG;
    doc["elMax"] = EL_MAX_DEG;
    doc["parkAz"] = PARK_AZ_DEG;
    doc["parkEl"] = PARK_EL_DEG;
    doc["azOffset"] = stepperControl.getAzimuthOffset();
    doc["elOffset"] = stepperControl.getElevationOffset();
    
    sendJSON(request, doc);
}

// =============================================================================
// BASIC CONTROL HANDLERS
// =============================================================================

void WebServer::handleSetPosition(AsyncWebServerRequest* request, JsonVariant& json) {
    if (!json.containsKey("azimuth") || !json.containsKey("elevation")) {
        sendError(request, "Missing azimuth or elevation");
        return;
    }
    
    float az = json["azimuth"].as<float>();
    float el = json["elevation"].as<float>();
    float speed = json["speed"] | TRACKING_SPEED_DEG_S;
    
    stepperControl.moveTo(az, el, speed);
    sendOK(request, "Position set");
}

void WebServer::handleStop(AsyncWebServerRequest* request) {
    stepperControl.stop();
    trackingEngine.stopTracking();
    sendOK(request, "Stopped");
}

void WebServer::handlePark(AsyncWebServerRequest* request) {
    stepperControl.park();
    sendOK(request, "Parking");
}

void WebServer::handleHome(AsyncWebServerRequest* request) {
    if (stepperControl.home()) {
        sendOK(request, "Homing complete");
    } else {
        sendError(request, "Homing failed");
    }
}

void WebServer::handleSetHome(AsyncWebServerRequest* request) {
    stepperControl.setHome();
    sendOK(request, "Home position set");
}

void WebServer::handleCalibrate(AsyncWebServerRequest* request, JsonVariant& json) {
    String method = json["method"] | "manual";
    
    if (method == "manual") {
        float azOffset = json["azOffset"] | 0.0f;
        float elOffset = json["elOffset"] | 0.0f;
        stepperControl.setCalibrationOffset(azOffset, elOffset);
        nvsStorage.saveCalibration(azOffset, elOffset);
        sendOK(request, "Manual calibration set");
    } else {
        sendError(request, "Only manual calibration is supported");
    }
}

// =============================================================================
// WIFI HANDLERS
// =============================================================================

void WebServer::handleGetNetworks(AsyncWebServerRequest* request) {
    NetworkInfo networks[20];
    int count = wifiManager.scanNetworks(networks, 20);
    
    StaticJsonDocument<JSON_SIZE_XLARGE> doc;
    JsonArray arr = doc["networks"].to<JsonArray>();
    
    for (int i = 0; i < count; i++) {
        JsonObject net = arr.createNestedObject();
        net["ssid"] = networks[i].ssid;
        net["rssi"] = networks[i].rssi;
        net["open"] = networks[i].open;
    }
    
    doc["count"] = count;
    sendJSON(request, doc);
}

void WebServer::handleConnectWiFi(AsyncWebServerRequest* request, JsonVariant& json) {
    if (!json.containsKey("ssid")) {
        sendError(request, "Missing SSID");
        return;
    }
    
    String ssid = json["ssid"].as<String>();
    String password = json["password"] | "";
    
    wifiManager.connect(ssid.c_str(), password.c_str(), true);
    sendOK(request, "Connecting...");
}

void WebServer::handleGetWiFiStatus(AsyncWebServerRequest* request) {
    StaticJsonDocument<JSON_SIZE_MEDIUM> doc;
    
    doc["mode"] = (int)wifiManager.getMode();
    doc["connected"] = wifiManager.isConnected();
    doc["ssid"] = wifiManager.getSSID();
    doc["ip"] = wifiManager.getIP().toString();
    doc["rssi"] = wifiManager.getRSSI();
    doc["hostname"] = wifiManager.getHostname();
    doc["mdns"] = wifiManager.isMDNSActive();
    
    sendJSON(request, doc);
}

// =============================================================================
// SYSTEM HANDLERS
// =============================================================================

void WebServer::handleGetSystemInfo(AsyncWebServerRequest* request) {
    StaticJsonDocument<JSON_SIZE_LARGE> doc;
    
    doc["version"] = FIRMWARE_VERSION;
    doc["model"] = DEVICE_MODEL;
    doc["freeHeap"] = ESP.getFreeHeap();
    doc["heapSize"] = ESP.getHeapSize();
    doc["freePsram"] = ESP.getFreePsram();
    doc["uptime"] = millis() / 1000;
    doc["cpuFreq"] = ESP.getCpuFreqMHz();
    doc["flashSize"] = ESP.getFlashChipSize();
    doc["sdkVersion"] = ESP.getSdkVersion();
    
    sendJSON(request, doc);
}

void WebServer::handleReboot(AsyncWebServerRequest* request) {
    sendOK(request, "Rebooting...");
    delay(500);
    ESP.restart();
}

void WebServer::handleFactoryReset(AsyncWebServerRequest* request) {
    nvsStorage.factoryReset();
    sendOK(request, "Factory reset complete. Rebooting...");
    delay(500);
    ESP.restart();
}

void WebServer::handleSetConfig(AsyncWebServerRequest* request, JsonVariant& json) {
    // Handle configuration updates
    if (json.containsKey("azOffset")) {
        float offset = json["azOffset"].as<float>();
        stepperControl.setCalibrationOffset(offset, stepperControl.getElevationOffset());
    }
    if (json.containsKey("elOffset")) {
        float offset = json["elOffset"].as<float>();
        stepperControl.setCalibrationOffset(stepperControl.getAzimuthOffset(), offset);
    }
    
    sendOK(request, "Configuration updated");
}

// =============================================================================
// GPS HANDLERS
// =============================================================================

void WebServer::handleSetManualGPS(AsyncWebServerRequest* request, JsonVariant& json) {
    if (!json.containsKey("latitude") || !json.containsKey("longitude")) {
        sendError(request, "Missing latitude or longitude");
        return;
    }
    
    double lat = json["latitude"].as<double>();
    double lon = json["longitude"].as<double>();
    double alt = json["altitude"] | 0.0;
    
    // Validate coordinates
    if (lat < -90 || lat > 90) {
        sendError(request, "Invalid latitude");
        return;
    }
    if (lon < -180 || lon > 180) {
        sendError(request, "Invalid longitude");
        return;
    }
    
    // Set manual position
    gps.setManualPosition(lat, lon, alt);
    gps.setUseManualPosition(true);
    
    // Update tracking engine
    trackingEngine.setObserverLocation(lat, lon, alt);
    
    // Save to NVS
    nvsStorage.saveLocation(lat, lon, alt, true);
    
    sendOK(request, "Manual position set");
}

// =============================================================================
// HELPER METHODS
// =============================================================================

void WebServer::sendJSON(AsyncWebServerRequest* request, JsonDocument& doc, int code) {
    String response;
    serializeJson(doc, response);
    request->send(code, "application/json", response);
}

void WebServer::sendError(AsyncWebServerRequest* request, const char* message, int code) {
    StaticJsonDocument<JSON_SIZE_SMALL> doc;
    doc["error"] = message;
    sendJSON(request, doc, code);
}

void WebServer::sendOK(AsyncWebServerRequest* request, const char* message) {
    StaticJsonDocument<JSON_SIZE_SMALL> doc;
    doc["status"] = "ok";
    doc["message"] = message;
    sendJSON(request, doc);
}

const char* WebServer::buildStatusJSON() {
    StaticJsonDocument<JSON_SIZE_MEDIUM> doc;
    
    doc["type"] = "status";
    doc["az"] = stepperControl.getAzimuth();
    doc["el"] = stepperControl.getElevation();
    doc["targetAz"] = stepperControl.getTargetAzimuth();
    doc["targetEl"] = stepperControl.getTargetElevation();
    doc["moving"] = stepperControl.isMoving();
    doc["mode"] = (int)trackingEngine.getMode();
    doc["sat"] = trackingEngine.getSatelliteName();
    doc["visible"] = trackingEngine.isSatelliteVisible();
    doc["gps"] = gps.hasFix();
    doc["time"] = gps.getUnixTime();
    
    if (trackingEngine.hasTLE()) {
        SatellitePosition pos = trackingEngine.getPosition();
        doc["satAz"] = pos.azimuth;
        doc["satEl"] = pos.elevation;
    }
    
    // Serialize to pre-allocated buffer instead of String
    size_t len = serializeJson(doc, _statusJsonBuffer, STATUS_JSON_BUFFER_SIZE);
    if (len >= STATUS_JSON_BUFFER_SIZE) {
        // Truncation occurred - should not happen with current data
        DEBUG_PRINTLN("WebServer: Status JSON truncated!");
    }
    return _statusJsonBuffer;
}

const char* WebServer::buildPassJSON(const PassInfo& pass) {
    StaticJsonDocument<JSON_SIZE_MEDIUM> doc;
    doc["aosTime"] = pass.aosTime;
    doc["losTime"] = pass.losTime;
    doc["maxElTime"] = pass.maxElTime;
    doc["aosAz"] = pass.aosAz;
    doc["losAz"] = pass.losAz;
    doc["maxEl"] = pass.maxEl;
    doc["maxElAz"] = pass.maxElAz;
    doc["duration"] = pass.losTime - pass.aosTime;
    
    // Serialize to pre-allocated buffer instead of String
    size_t len = serializeJson(doc, _passJsonBuffer, PASS_JSON_BUFFER_SIZE);
    if (len >= PASS_JSON_BUFFER_SIZE) {
        DEBUG_PRINTLN("WebServer: Pass JSON truncated!");
    }
    return _passJsonBuffer;
}
