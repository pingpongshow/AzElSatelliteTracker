/**
 * @file rotctld_server.h
 * @brief Hamlib rotctld protocol server for GPredict compatibility
 * 
 * Implements the rotctld TCP protocol used by GPredict and other
 * satellite tracking software to control antenna rotators.
 */

#ifndef ROTCTLD_SERVER_H
#define ROTCTLD_SERVER_H

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include "config.h"

/**
 * @class RotctldServer
 * @brief TCP server implementing Hamlib rotctld protocol
 * 
 * Protocol commands supported:
 * - p, \get_pos      : Get current azimuth and elevation
 * - P, \set_pos      : Set azimuth and elevation
 * - S, \stop         : Stop rotation
 * - K, \park         : Move to park position
 * - M, \move         : Move in direction at speed
 * - q, \quit         : Close connection
 * - _, \get_info     : Get rotator info
 * - \dump_caps       : Get capabilities
 */
class RotctldServer {
public:
    RotctldServer();
    ~RotctldServer();
    
    /**
     * @brief Start the rotctld server
     * @param port TCP port (default 4533)
     * @return true if started successfully
     */
    bool begin(uint16_t port = ROTCTLD_PORT);
    
    /**
     * @brief Stop the server
     */
    void stop();
    
    /**
     * @brief Update server (call from main loop)
     */
    void update();
    
    /**
     * @brief Check if server is running
     */
    bool isRunning() const { return _running; }
    
    /**
     * @brief Get number of connected clients
     */
    int getClientCount() const { return _clientCount; }
    
    /**
     * @brief Check if external control is active
     */
    bool isExternalControlActive() const { return _externalControl; }
    
    /**
     * @brief Set callback for position change
     */
    void onPositionChange(void (*callback)(float az, float el)) {
        _positionCallback = callback;
    }
    
    /**
     * @brief Set callback for stop command
     */
    void onStop(void (*callback)()) {
        _stopCallback = callback;
    }
    
    /**
     * @brief Set callback for park command
     */
    void onPark(void (*callback)()) {
        _parkCallback = callback;
    }

private:
    AsyncServer* _server;
    AsyncClient* _clients[ROTCTLD_MAX_CLIENTS];
    int _clientCount;
    bool _running;
    bool _externalControl;
    
    // Command buffer per client
    char _cmdBuffer[ROTCTLD_MAX_CLIENTS][ROTCTLD_BUFFER_SIZE];
    int _cmdLength[ROTCTLD_MAX_CLIENTS];
    
    // Multi-line command state (for GPredict which sends "P az\nel")
    bool _waitingForElevation[ROTCTLD_MAX_CLIENTS];
    float _pendingAzimuth[ROTCTLD_MAX_CLIENTS];
    
    // Callbacks
    void (*_positionCallback)(float az, float el);
    void (*_stopCallback)();
    void (*_parkCallback)();
    
    // Client management
    void handleNewClient(AsyncClient* client);
    void handleClientData(AsyncClient* client, void* data, size_t len);
    void handleClientDisconnect(AsyncClient* client);
    int findClientIndex(AsyncClient* client);
    
    // Command processing
    void processCommand(AsyncClient* client, const char* cmd);
    void handleGetPos(AsyncClient* client);
    void handleSetPos(AsyncClient* client, const char* args);
    void handleStop(AsyncClient* client);
    void handlePark(AsyncClient* client);
    void handleMove(AsyncClient* client, const char* args);
    void handleGetInfo(AsyncClient* client);
    void handleDumpCaps(AsyncClient* client);
    
    // Response helpers
    void sendResponse(AsyncClient* client, const char* response);
    void sendOK(AsyncClient* client);
    void sendError(AsyncClient* client, const char* msg);
};

// Global instance
extern RotctldServer rotctldServer;

#endif // ROTCTLD_SERVER_H
