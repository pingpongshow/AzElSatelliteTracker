/**
 * @file web_server.h
 * @brief Web server for UI and REST API
 * 
 * Provides web-based user interface and REST API endpoints
 * for controlling the satellite tracker.
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>  // For AsyncCallbackJsonWebHandler
#include <ArduinoJson.h>
#include <vector>
#include "config.h"

// Forward declaration for PassInfo (defined in tracking_engine.h)
struct PassInfo;

/**
 * @class WebServer
 * @brief Async web server with REST API
 */
class WebServer {
public:
    WebServer();
    ~WebServer();
    
    /**
     * @brief Initialize and start web server
     * @return true if started successfully
     */
    bool begin();
    
    /**
     * @brief Stop web server
     */
    void stop();
    
    /**
     * @brief Update server (broadcast status via WebSocket)
     */
    void update();
    
    /**
     * @brief Check if server is running
     */
    bool isRunning() const { return _running; }
    
    /**
     * @brief Broadcast status to all WebSocket clients
     */
    void broadcastStatus();
    
    /**
     * @brief Send event to WebSocket clients
     * @param event Event name
     * @param data Event data as JSON string
     */
    void sendEvent(const char* event, const char* data);
    
    /**
     * @brief Get number of connected WebSocket clients
     */
    int getClientCount() const;

private:
    AsyncWebServer* _server;
    AsyncWebSocket* _ws;
    std::vector<AsyncCallbackJsonWebHandler*> _jsonHandlers;  // Track JSON handlers for cleanup
    bool _running;
    bool _spiffsAvailable;  // Whether SPIFFS is mounted
    uint32_t _lastBroadcast;
    
    // Pre-allocated buffer for status JSON to avoid heap fragmentation
    static const size_t STATUS_JSON_BUFFER_SIZE = 512;
    char _statusJsonBuffer[STATUS_JSON_BUFFER_SIZE];
    
    // Pre-allocated buffer for pass JSON to avoid heap allocations in hot paths
    static const size_t PASS_JSON_BUFFER_SIZE = 256;
    char _passJsonBuffer[PASS_JSON_BUFFER_SIZE];
    
    // Route setup
    void setupRoutes();
    void setupAPIRoutes();
    void setupAdvancedRoutes();
    void setupWebSocketHandler();
    
    // Static file handlers
    void handleRoot(AsyncWebServerRequest* request);
    void handleNotFound(AsyncWebServerRequest* request);
    
    // API handlers - Status
    void handleGetStatus(AsyncWebServerRequest* request);
    void handleGetPosition(AsyncWebServerRequest* request);
    void handleGetGPS(AsyncWebServerRequest* request);
    void handleGetConfig(AsyncWebServerRequest* request);
    
    // API handlers - Control
    void handleSetPosition(AsyncWebServerRequest* request, JsonVariant& json);
    void handleStop(AsyncWebServerRequest* request);
    void handlePark(AsyncWebServerRequest* request);
    void handleHome(AsyncWebServerRequest* request);
    void handleSetHome(AsyncWebServerRequest* request);
    void handleCalibrate(AsyncWebServerRequest* request, JsonVariant& json);
    
    // API handlers - Tracking
    void handleStartTracking(AsyncWebServerRequest* request, JsonVariant& json);
    void handleStopTracking(AsyncWebServerRequest* request);
    void handleGetNextPass(AsyncWebServerRequest* request);
    
    // API handlers - TLE
    void handleGetTLEs(AsyncWebServerRequest* request);
    void handleLoadTLE(AsyncWebServerRequest* request, JsonVariant& json);
    void handleFetchTLE(AsyncWebServerRequest* request, JsonVariant& json);
    void handleDeleteTLE(AsyncWebServerRequest* request, JsonVariant& json);
    
    // API handlers - WiFi
    void handleGetNetworks(AsyncWebServerRequest* request);
    void handleConnectWiFi(AsyncWebServerRequest* request, JsonVariant& json);
    void handleGetWiFiStatus(AsyncWebServerRequest* request);
    
    // API handlers - System
    void handleReboot(AsyncWebServerRequest* request);
    void handleFactoryReset(AsyncWebServerRequest* request);
    void handleGetSystemInfo(AsyncWebServerRequest* request);
    void handleSetConfig(AsyncWebServerRequest* request, JsonVariant& json);
    
    // API handlers - GPS
    void handleSetManualGPS(AsyncWebServerRequest* request, JsonVariant& json);
    
    // =========================================================================
    // Multi-pass prediction handlers
    // =========================================================================
    void handleGetPasses(AsyncWebServerRequest* request);
    void handleSchedulePass(AsyncWebServerRequest* request, JsonVariant& json);
    void handleCancelPass(AsyncWebServerRequest* request, JsonVariant& json);
    void handleGetSchedule(AsyncWebServerRequest* request);
    void handleClearSchedule(AsyncWebServerRequest* request);
    
    // =========================================================================
    // Doppler calculation handlers
    // =========================================================================
    void handleGetDoppler(AsyncWebServerRequest* request);
    
    // =========================================================================
    // Health monitoring handlers
    // =========================================================================
    void handleGetHealth(AsyncWebServerRequest* request);
    void handleGetHealthEvents(AsyncWebServerRequest* request);
    void handleClearHealthEvents(AsyncWebServerRequest* request);
    
    // =========================================================================
    // SD card handlers
    // =========================================================================
    void handleSDStatus(AsyncWebServerRequest* request);
    void handleSDListTLEs(AsyncWebServerRequest* request);
    void handleSDImportTLE(AsyncWebServerRequest* request, JsonVariant& json);
    void handleSDExportTLEs(AsyncWebServerRequest* request);
    void handleSDBackupConfig(AsyncWebServerRequest* request);
    void handleSDRestoreConfig(AsyncWebServerRequest* request);
    void handleGetPassLog(AsyncWebServerRequest* request);
    void handleExportPassLog(AsyncWebServerRequest* request);
    
    // =========================================================================
    // External API handlers
    // =========================================================================
    void handleSetN2YOKey(AsyncWebServerRequest* request, JsonVariant& json);
    void handleFetchN2YOTLE(AsyncWebServerRequest* request, JsonVariant& json);
    void handleGetVisualPasses(AsyncWebServerRequest* request);
    void handleSearchSatNOGS(AsyncWebServerRequest* request);
    void handleGetTransmitters(AsyncWebServerRequest* request);
    void handleFetchTLEGroup(AsyncWebServerRequest* request, JsonVariant& json);
    
    // =========================================================================
    // UDP broadcast handlers
    // =========================================================================
    void handleUDPConfig(AsyncWebServerRequest* request, JsonVariant& json);
    void handleUDPStatus(AsyncWebServerRequest* request);
    
    // =========================================================================
    // EME mode handlers
    // =========================================================================
    void handleEnableEME(AsyncWebServerRequest* request);
    void handleDisableEME(AsyncWebServerRequest* request);
    void handleGetMoonInfo(AsyncWebServerRequest* request);
    void handleGetEMEPathLoss(AsyncWebServerRequest* request);
    
    // =========================================================================
    // Geo satellite handlers
    // =========================================================================
    void handlePointToGeo(AsyncWebServerRequest* request, JsonVariant& json);
    void handleEnableGeoTracking(AsyncWebServerRequest* request, JsonVariant& json);
    void handleDisableGeoTracking(AsyncWebServerRequest* request);
    
    // =========================================================================
    // Antenna pattern handlers
    // =========================================================================
    void handleGetAntennaPattern(AsyncWebServerRequest* request);
    void handleSetAntennaPattern(AsyncWebServerRequest* request, JsonVariant& json);
    
    // =========================================================================
    // Pre-positioning handlers
    // =========================================================================
    void handleSetPrePosition(AsyncWebServerRequest* request, JsonVariant& json);
    
    // WebSocket handler
    void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                          AwsEventType type, void* arg, uint8_t* data, size_t len);
    
    // Helper methods
    void sendJSON(AsyncWebServerRequest* request, JsonDocument& doc, int code = 200);
    void sendError(AsyncWebServerRequest* request, const char* message, int code = 400);
    void sendOK(AsyncWebServerRequest* request, const char* message = "OK");
    const char* buildStatusJSON();  // Returns pointer to pre-allocated buffer
    const char* buildPassJSON(const PassInfo& pass);
};

// Global instance
extern WebServer webServer;

#endif // WEB_SERVER_H
