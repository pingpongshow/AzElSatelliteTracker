/**
 * @file udp_broadcast.cpp
 * @brief UDP position broadcast implementation
 */

#include "udp_broadcast.h"
#include "stepper_control.h"
#include "tracking_engine.h"
#include <WiFi.h>

// Global instance
UDPBroadcast udpBroadcast;

UDPBroadcast::UDPBroadcast()
    : _enabled(false)
    , _running(false)
    , _port(UDP_BROADCAST_PORT)
    , _format(UDPFormat::JSON)
    , _useMulticast(false)
    , _intervalMs(UDP_BROADCAST_INTERVAL_MS)
    , _lastBroadcast(0)
    , _broadcastCount(0)
    , _clientCount(0)
{
    _destIP = IPAddress(255, 255, 255, 255);  // Broadcast by default
}

bool UDPBroadcast::begin(uint16_t port, UDPFormat format) {
    DEBUG_PRINTLN("UDPBroadcast: Initializing...");
    
    _port = port;
    _format = format;
    
    if (!_udp.begin(_port)) {
        DEBUG_PRINTLN("UDPBroadcast: Failed to start UDP");
        return false;
    }
    
    _running = true;
    _enabled = true;
    
    DEBUG_PRINTF("UDPBroadcast: Started on port %d\n", _port);
    return true;
}

void UDPBroadcast::stop() {
    _udp.stop();
    _running = false;
    _enabled = false;
    DEBUG_PRINTLN("UDPBroadcast: Stopped");
}

void UDPBroadcast::update() {
    if (!_running || !_enabled) return;
    
    uint32_t now = millis();
    if (now - _lastBroadcast >= _intervalMs) {
        broadcastNow();
        _lastBroadcast = now;
    }
}

void UDPBroadcast::broadcastNow() {
    if (!_running || !_enabled) return;
    
    String data;
    
    switch (_format) {
        case UDPFormat::JSON:
            data = formatJSON();
            break;
        case UDPFormat::CSV:
            data = formatCSV();
            break;
        case UDPFormat::EASYCOMM:
            data = formatEasycomm();
            break;
        case UDPFormat::SIMPLE:
            data = formatSimple();
            break;
    }
    
    sendPacket(data);
    _broadcastCount++;
}

String UDPBroadcast::formatJSON() {
    float az = stepperControl.getAzimuth();
    float el = stepperControl.getElevation();
    bool moving = stepperControl.isMoving();
    
    String json = "{";
    json += "\"az\":" + String(az, 2) + ",";
    json += "\"el\":" + String(el, 2) + ",";
    json += "\"moving\":" + String(moving ? "true" : "false");
    
    if (trackingEngine.hasTLE()) {
        json += ",\"sat\":\"" + trackingEngine.getSatelliteName() + "\"";
        
        if (trackingEngine.isSatelliteVisible()) {
            SatellitePosition pos = trackingEngine.getPosition();
            json += ",\"satAz\":" + String(pos.azimuth, 2);
            json += ",\"satEl\":" + String(pos.elevation, 2);
            json += ",\"range\":" + String(pos.range, 1);
        }
        json += ",\"visible\":" + String(trackingEngine.isSatelliteVisible() ? "true" : "false");
    }
    
    json += ",\"mode\":" + String((int)trackingEngine.getMode());
    json += "}\n";
    
    return json;
}

String UDPBroadcast::formatCSV() {
    float az = stepperControl.getAzimuth();
    float el = stepperControl.getElevation();
    
    String csv = String(az, 2) + "," + String(el, 2);
    
    if (trackingEngine.hasTLE()) {
        csv += "," + trackingEngine.getSatelliteName();
        
        if (trackingEngine.isSatelliteVisible()) {
            SatellitePosition pos = trackingEngine.getPosition();
            csv += "," + String(pos.azimuth, 2);
            csv += "," + String(pos.elevation, 2);
        }
    }
    
    csv += "\n";
    return csv;
}

String UDPBroadcast::formatEasycomm() {
    // EasyComm II format: AZaaa.a ELeee.e
    float az = stepperControl.getAzimuth();
    float el = stepperControl.getElevation();
    
    char buf[32];
    snprintf(buf, sizeof(buf), "AZ%.1f EL%.1f\n", az, el);
    return String(buf);
}

String UDPBroadcast::formatSimple() {
    float az = stepperControl.getAzimuth();
    float el = stepperControl.getElevation();
    
    return String(az, 2) + " " + String(el, 2) + "\n";
}

void UDPBroadcast::sendPacket(const String& data) {
    if (_useMulticast) {
        // Send to multicast group - use beginPacket with multicast address
        // Note: For proper multicast, call beginMulticast() in begin() to join the group
        _udp.beginPacket(_multicastIP, _port);
        _udp.write((const uint8_t*)data.c_str(), data.length());
        _udp.endPacket();
    } else if (_clientCount > 0) {
        // Send to each registered client
        for (int i = 0; i < _clientCount; i++) {
            _udp.beginPacket(_clients[i], _port);
            _udp.write((const uint8_t*)data.c_str(), data.length());
            _udp.endPacket();
        }
    } else {
        // Broadcast
        _udp.beginPacket(_destIP, _port);
        _udp.write((const uint8_t*)data.c_str(), data.length());
        _udp.endPacket();
    }
}

void UDPBroadcast::setMulticast(IPAddress multicastIP) {
    _multicastIP = multicastIP;
    _useMulticast = true;
    DEBUG_PRINTF("UDPBroadcast: Multicast enabled to %s\n", multicastIP.toString().c_str());
}

void UDPBroadcast::addClient(IPAddress ip) {
    if (_clientCount >= MAX_CLIENTS) {
        DEBUG_PRINTLN("UDPBroadcast: Max clients reached");
        return;
    }
    
    // Check if already added
    for (int i = 0; i < _clientCount; i++) {
        if (_clients[i] == ip) return;
    }
    
    _clients[_clientCount++] = ip;
    _useMulticast = false;  // Switch to unicast mode
    DEBUG_PRINTF("UDPBroadcast: Added client %s\n", ip.toString().c_str());
}

void UDPBroadcast::removeClient(IPAddress ip) {
    for (int i = 0; i < _clientCount; i++) {
        if (_clients[i] == ip) {
            // Shift remaining clients
            for (int j = i; j < _clientCount - 1; j++) {
                _clients[j] = _clients[j + 1];
            }
            _clientCount--;
            DEBUG_PRINTF("UDPBroadcast: Removed client %s\n", ip.toString().c_str());
            return;
        }
    }
}
