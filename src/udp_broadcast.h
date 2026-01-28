/**
 * @file udp_broadcast.h
 * @brief UDP position broadcast for GPredict and other clients
 * 
 * Broadcasts antenna position via UDP for integration with
 * external tracking software that supports UDP position input.
 */

#ifndef UDP_BROADCAST_H
#define UDP_BROADCAST_H

#include <Arduino.h>
#include <WiFiUdp.h>
#include "config.h"

// Broadcast format types
enum class UDPFormat {
    JSON,           // {"az":123.4,"el":45.6,"sat":"ISS"}
    CSV,            // 123.4,45.6,ISS
    EASYCOMM,       // AZ123.4 EL45.6
    SIMPLE          // 123.4 45.6
};

/**
 * @class UDPBroadcast
 * @brief UDP position broadcaster
 */
class UDPBroadcast {
public:
    UDPBroadcast();
    
    /**
     * @brief Initialize UDP broadcast
     * @param port UDP port to broadcast on
     * @param format Broadcast format
     * @return true if initialized
     */
    bool begin(uint16_t port = UDP_BROADCAST_PORT, UDPFormat format = UDPFormat::JSON);
    
    /**
     * @brief Stop broadcasting
     */
    void stop();
    
    /**
     * @brief Update - sends broadcast if interval elapsed
     * Call from main loop
     */
    void update();
    
    /**
     * @brief Force immediate broadcast
     */
    void broadcastNow();
    
    /**
     * @brief Enable/disable broadcasting
     */
    void setEnabled(bool enabled) { _enabled = enabled; }
    
    /**
     * @brief Check if broadcasting is enabled
     */
    bool isEnabled() const { return _enabled; }
    
    /**
     * @brief Set broadcast interval
     * @param intervalMs Milliseconds between broadcasts
     */
    void setInterval(uint32_t intervalMs) { _intervalMs = intervalMs; }
    
    /**
     * @brief Set broadcast format
     */
    void setFormat(UDPFormat format) { _format = format; }
    
    /**
     * @brief Set broadcast destination
     * @param ip Destination IP (or broadcast address)
     */
    void setDestination(IPAddress ip) { _destIP = ip; }
    
    /**
     * @brief Enable multicast
     * @param multicastIP Multicast group address
     */
    void setMulticast(IPAddress multicastIP);
    
    /**
     * @brief Add client to unicast list
     * @param ip Client IP address
     */
    void addClient(IPAddress ip);
    
    /**
     * @brief Remove client from unicast list
     */
    void removeClient(IPAddress ip);
    
    /**
     * @brief Get number of broadcasts sent
     */
    uint32_t getBroadcastCount() const { return _broadcastCount; }
    
    /**
     * @brief Get broadcast port
     */
    uint16_t getPort() const { return _port; }

private:
    WiFiUDP _udp;
    bool _enabled;
    bool _running;
    uint16_t _port;
    UDPFormat _format;
    IPAddress _destIP;
    bool _useMulticast;
    IPAddress _multicastIP;
    uint32_t _intervalMs;
    uint32_t _lastBroadcast;
    uint32_t _broadcastCount;
    
    // Unicast client list
    static const int MAX_CLIENTS = 5;
    IPAddress _clients[MAX_CLIENTS];
    int _clientCount;
    
    // Format methods
    String formatJSON();
    String formatCSV();
    String formatEasycomm();
    String formatSimple();
    
    void sendPacket(const String& data);
};

// Global instance
extern UDPBroadcast udpBroadcast;

#endif // UDP_BROADCAST_H
