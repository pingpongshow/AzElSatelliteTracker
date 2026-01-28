/**
 * @file rotctld_server.cpp
 * @brief Hamlib rotctld protocol server implementation
 */

#include "rotctld_server.h"
#include "stepper_control.h"
#include "tracking_engine.h"

// Global instance
RotctldServer rotctldServer;

RotctldServer::RotctldServer()
    : _server(nullptr)
    , _clientCount(0)
    , _running(false)
    , _externalControl(false)
    , _positionCallback(nullptr)
    , _stopCallback(nullptr)
    , _parkCallback(nullptr)
{
    for (int i = 0; i < ROTCTLD_MAX_CLIENTS; i++) {
        _clients[i] = nullptr;
        _cmdLength[i] = 0;
        _waitingForElevation[i] = false;
        _pendingAzimuth[i] = 0;
    }
}

RotctldServer::~RotctldServer() {
    stop();
}

bool RotctldServer::begin(uint16_t port) {
    if (_running) {
        stop();
    }
    
    DEBUG_PRINTF("RotctldServer: Starting on port %d\n", port);
    
    _server = new AsyncServer(port);
    
    _server->onClient([this](void* arg, AsyncClient* client) {
        handleNewClient(client);
    }, nullptr);
    
    _server->begin();
    _running = true;
    
    DEBUG_PRINTLN("RotctldServer: Started");
    return true;
}

void RotctldServer::stop() {
    if (!_running) return;
    
    // Disconnect all clients
    for (int i = 0; i < ROTCTLD_MAX_CLIENTS; i++) {
        if (_clients[i]) {
            _clients[i]->close();
            _clients[i] = nullptr;
        }
    }
    _clientCount = 0;
    
    // Stop server
    if (_server) {
        _server->end();
        delete _server;
        _server = nullptr;
    }
    
    _running = false;
    _externalControl = false;
    
    DEBUG_PRINTLN("RotctldServer: Stopped");
}

void RotctldServer::update() {
    // AsyncTCP handles everything in callbacks
    // This method is available for any periodic tasks
}

void RotctldServer::handleNewClient(AsyncClient* client) {
    if (_clientCount >= ROTCTLD_MAX_CLIENTS) {
        DEBUG_PRINTLN("RotctldServer: Max clients reached, rejecting");
        client->close();
        return;
    }
    
    // Find empty slot
    int slot = -1;
    for (int i = 0; i < ROTCTLD_MAX_CLIENTS; i++) {
        if (_clients[i] == nullptr) {
            slot = i;
            break;
        }
    }
    
    if (slot < 0) {
        client->close();
        return;
    }
    
    _clients[slot] = client;
    _cmdLength[slot] = 0;
    _waitingForElevation[slot] = false;
    _pendingAzimuth[slot] = 0;
    _clientCount++;
    
    DEBUG_PRINTF("RotctldServer: Client connected from %s (slot %d)\n",
                 client->remoteIP().toString().c_str(), slot);
    
    // Set up callbacks
    client->onData([this](void* arg, AsyncClient* c, void* data, size_t len) {
        handleClientData(c, data, len);
    }, nullptr);
    
    client->onDisconnect([this](void* arg, AsyncClient* c) {
        handleClientDisconnect(c);
    }, nullptr);
    
    client->onError([this](void* arg, AsyncClient* c, int8_t error) {
        DEBUG_PRINTF("RotctldServer: Client error %d\n", error);
    }, nullptr);
    
    // Enable external control mode
    _externalControl = true;
    trackingEngine.setMode(TrackingMode::ROTCTLD);
}

void RotctldServer::handleClientData(AsyncClient* client, void* data, size_t len) {
    int slot = findClientIndex(client);
    if (slot < 0) return;
    
    char* buf = (char*)data;
    
    // Debug: print raw received data
    DEBUG_PRINTF("RotctldServer: Received %d bytes: ", len);
    for (size_t i = 0; i < len && i < 64; i++) {
        if (buf[i] >= 32 && buf[i] < 127) {
            DEBUG_PRINTF("%c", buf[i]);
        } else {
            DEBUG_PRINTF("[0x%02X]", (uint8_t)buf[i]);
        }
    }
    DEBUG_PRINTLN("");
    
    for (size_t i = 0; i < len; i++) {
        char c = buf[i];
        
        if (c == '\n' || c == '\r') {
            // End of command
            if (_cmdLength[slot] > 0) {
                _cmdBuffer[slot][_cmdLength[slot]] = '\0';
                processCommand(client, _cmdBuffer[slot]);
                _cmdLength[slot] = 0;
            }
        } else if (_cmdLength[slot] < ROTCTLD_BUFFER_SIZE - 1) {
            _cmdBuffer[slot][_cmdLength[slot]++] = c;
        }
    }
}

void RotctldServer::handleClientDisconnect(AsyncClient* client) {
    int slot = findClientIndex(client);
    if (slot >= 0) {
        _clients[slot] = nullptr;
        _cmdLength[slot] = 0;
        _waitingForElevation[slot] = false;
        _pendingAzimuth[slot] = 0;
        _clientCount--;
        DEBUG_PRINTF("RotctldServer: Client disconnected (slot %d)\n", slot);
    }
    
    // Disable external control if no clients
    if (_clientCount == 0) {
        _externalControl = false;
        if (trackingEngine.getMode() == TrackingMode::ROTCTLD) {
            trackingEngine.setMode(TrackingMode::IDLE);
        }
    }
}

int RotctldServer::findClientIndex(AsyncClient* client) {
    for (int i = 0; i < ROTCTLD_MAX_CLIENTS; i++) {
        if (_clients[i] == client) {
            return i;
        }
    }
    return -1;
}

void RotctldServer::processCommand(AsyncClient* client, const char* cmd) {
    int slot = findClientIndex(client);
    if (slot < 0) return;
    
    DEBUG_PRINTF("RotctldServer: Command '%s' (slot %d)\n", cmd, slot);
    
    // Skip leading whitespace
    while (*cmd == ' ' || *cmd == '\t') cmd++;
    
    // Empty command
    if (cmd[0] == '\0') {
        return;
    }
    
    // Check if we're waiting for elevation (multi-line P command from GPredict)
    if (_waitingForElevation[slot]) {
        float el;
        if (sscanf(cmd, "%f", &el) == 1) {
            float az = _pendingAzimuth[slot];
            _waitingForElevation[slot] = false;
            
            DEBUG_PRINTF("RotctldServer: Got elevation %.2f, completing move to Az=%.2f El=%.2f\n", el, az, el);
            
            // Validate ranges
            if (az < AZ_MIN_DEG || az > AZ_MAX_DEG) {
                DEBUG_PRINTF("RotctldServer: Az %.2f out of range\n", az);
                sendError(client, "Azimuth out of range");
                return;
            }
            if (el < EL_MIN_DEG || el > EL_MAX_DEG) {
                DEBUG_PRINTF("RotctldServer: El %.2f out of range\n", el);
                sendError(client, "Elevation out of range");
                return;
            }
            
            // Execute move
            stepperControl.moveTo(az, el, MAX_SLEW_SPEED_DEG_S);
            
            if (_positionCallback) {
                _positionCallback(az, el);
            }
            
            sendOK(client);
            return;
        } else {
            // Not a number - cancel waiting state and process as new command
            _waitingForElevation[slot] = false;
            DEBUG_PRINTLN("RotctldServer: Expected elevation, got other command");
        }
    }
    
    // Extended command format (backslash prefix)
    // Also handle '+' prefix (extended response protocol used by some clients)
    if (cmd[0] == '+') {
        cmd++;  // Skip the '+' prefix
    }
    
    if (cmd[0] == '\\') {
        const char* extCmd = cmd + 1;
        
        if (strncmp(extCmd, "get_pos", 7) == 0) {
            handleGetPos(client);
        } else if (strncmp(extCmd, "set_pos", 7) == 0) {
            handleSetPos(client, extCmd + 7);
        } else if (strncmp(extCmd, "stop", 4) == 0) {
            handleStop(client);
        } else if (strncmp(extCmd, "park", 4) == 0) {
            handlePark(client);
        } else if (strncmp(extCmd, "move", 4) == 0) {
            handleMove(client, extCmd + 4);
        } else if (strncmp(extCmd, "get_info", 8) == 0) {
            handleGetInfo(client);
        } else if (strncmp(extCmd, "dump_caps", 9) == 0) {
            handleDumpCaps(client);
        } else if (strncmp(extCmd, "quit", 4) == 0) {
            client->close();
        } else {
            sendError(client, "Unknown command");
        }
        return;
    }
    
    // Short command format
    switch (cmd[0]) {
        case 'p':
            handleGetPos(client);
            break;
            
        case 'P':
            handleSetPos(client, cmd + 1);
            break;
            
        case 'S':
            handleStop(client);
            break;
            
        case 'K':
            handlePark(client);
            break;
            
        case 'M':
            handleMove(client, cmd + 1);
            break;
            
        case 'q':
            client->close();
            break;
            
        case '_':
            handleGetInfo(client);
            break;
            
        default:
            sendError(client, "Unknown command");
            break;
    }
}

void RotctldServer::handleGetPos(AsyncClient* client) {
    float az = stepperControl.getAzimuth();
    float el = stepperControl.getElevation();
    
    char response[64];
    snprintf(response, sizeof(response), "%.6f\n%.6f\n", az, el);
    sendResponse(client, response);
}

void RotctldServer::handleSetPos(AsyncClient* client, const char* args) {
    int slot = findClientIndex(client);
    if (slot < 0) return;
    
    float az, el;
    
    DEBUG_PRINTF("RotctldServer: SetPos args='%s'\n", args);
    
    // Skip leading whitespace
    while (*args == ' ' || *args == '\t') args++;
    
    // Try to parse both values at once (space, newline, or comma separated)
    int parsed = sscanf(args, "%f %f", &az, &el);
    
    if (parsed != 2) {
        parsed = sscanf(args, "%f\n%f", &az, &el);
    }
    
    if (parsed != 2) {
        parsed = sscanf(args, "%f,%f", &az, &el);
    }
    
    // If we got both values, execute immediately
    if (parsed == 2) {
        DEBUG_PRINTF("RotctldServer: Parsed both Az=%.2f El=%.2f\n", az, el);
        
        // Validate ranges
        if (az < AZ_MIN_DEG || az > AZ_MAX_DEG) {
            DEBUG_PRINTF("RotctldServer: Az %.2f out of range\n", az);
            sendError(client, "Azimuth out of range");
            return;
        }
        if (el < EL_MIN_DEG || el > EL_MAX_DEG) {
            DEBUG_PRINTF("RotctldServer: El %.2f out of range\n", el);
            sendError(client, "Elevation out of range");
            return;
        }
        
        DEBUG_PRINTF("RotctldServer: Moving to Az=%.2f El=%.2f\n", az, el);
        
        stepperControl.moveTo(az, el, MAX_SLEW_SPEED_DEG_S);
        
        if (_positionCallback) {
            _positionCallback(az, el);
        }
        
        sendOK(client);
        return;
    }
    
    // If we got only one value (azimuth), wait for elevation on next line
    // This handles GPredict's format: "P az\nel"
    if (sscanf(args, "%f", &az) == 1) {
        DEBUG_PRINTF("RotctldServer: Got only azimuth %.2f, waiting for elevation\n", az);
        _pendingAzimuth[slot] = az;
        _waitingForElevation[slot] = true;
        // Don't send response yet - wait for elevation
        return;
    }
    
    // Couldn't parse anything
    DEBUG_PRINTLN("RotctldServer: Parse failed completely");
    sendError(client, "Invalid position format");
}

void RotctldServer::handleStop(AsyncClient* client) {
    stepperControl.stop();
    
    if (_stopCallback) {
        _stopCallback();
    }
    
    sendOK(client);
}

void RotctldServer::handlePark(AsyncClient* client) {
    stepperControl.park();
    
    if (_parkCallback) {
        _parkCallback();
    }
    
    sendOK(client);
}

void RotctldServer::handleMove(AsyncClient* client, const char* args) {
    int direction;
    int speed;
    
    if (sscanf(args, "%d %d", &direction, &speed) != 2) {
        sendError(client, "Invalid move format");
        return;
    }
    
    // Validate speed parameter to prevent dangerous values
    float validatedSpeed = (float)speed;
    if (validatedSpeed < ROTCTLD_MIN_SPEED_DEG_S) {
        validatedSpeed = ROTCTLD_MIN_SPEED_DEG_S;
    } else if (validatedSpeed > ROTCTLD_MAX_SPEED_DEG_S) {
        validatedSpeed = ROTCTLD_MAX_SPEED_DEG_S;
    }
    
    // Direction: 2=up, 4=down, 8=left, 16=right (Hamlib standard)
    float currentAz = stepperControl.getAzimuth();
    float currentEl = stepperControl.getElevation();
    float targetAz = currentAz;
    float targetEl = currentEl;
    float moveAmount = ROTCTLD_MOVE_AMOUNT_DEG;
    
    if (direction & ROTCTLD_DIR_UP) targetEl += moveAmount;
    if (direction & ROTCTLD_DIR_DOWN) targetEl -= moveAmount;
    if (direction & ROTCTLD_DIR_LEFT) targetAz -= moveAmount;
    if (direction & ROTCTLD_DIR_RIGHT) targetAz += moveAmount;
    
    // Clamp to limits
    if (targetEl < EL_MIN_DEG) targetEl = EL_MIN_DEG;
    if (targetEl > EL_MAX_DEG) targetEl = EL_MAX_DEG;
    
    stepperControl.moveTo(targetAz, targetEl, validatedSpeed);
    
    sendOK(client);
}

void RotctldServer::handleGetInfo(AsyncClient* client) {
    char response[256];
    snprintf(response, sizeof(response),
             "Model: %s %s\n"
             "Version: %s\n"
             "Az: %.1f to %.1f\n"
             "El: %.1f to %.1f\n",
             DEVICE_NAME, DEVICE_MODEL,
             FIRMWARE_VERSION,
             AZ_MIN_DEG, AZ_MAX_DEG,
             EL_MIN_DEG, EL_MAX_DEG);
    sendResponse(client, response);
}

void RotctldServer::handleDumpCaps(AsyncClient* client) {
    char response[512];
    snprintf(response, sizeof(response),
             "Caps dump for model: %s %s\n"
             "Mfg name: ESP32-DIY\n"
             "Model name: %s\n"
             "Version: %s\n"
             "Status: Beta\n"
             "Can set Position: Y\n"
             "Can get Position: Y\n"
             "Can stop: Y\n"
             "Can park: Y\n"
             "Min Azimuth: %.1f\n"
             "Max Azimuth: %.1f\n"
             "Min Elevation: %.1f\n"
             "Max Elevation: %.1f\n",
             DEVICE_NAME, DEVICE_MODEL,
             DEVICE_MODEL,
             FIRMWARE_VERSION,
             AZ_MIN_DEG, AZ_MAX_DEG,
             EL_MIN_DEG, EL_MAX_DEG);
    sendResponse(client, response);
}

void RotctldServer::sendResponse(AsyncClient* client, const char* response) {
    if (client && client->connected()) {
        client->write(response);
    }
}

void RotctldServer::sendOK(AsyncClient* client) {
    sendResponse(client, "RPRT 0\n");
}

void RotctldServer::sendError(AsyncClient* client, const char* msg) {
    char response[128];
    snprintf(response, sizeof(response), "RPRT -1\n");
    sendResponse(client, response);
    DEBUG_PRINTF("RotctldServer: Error - %s\n", msg);
}
