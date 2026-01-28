/**
 * @file wifi_manager.h
 * @brief WiFi connection management with AP fallback
 * 
 * Handles WiFi station mode connection to home network with
 * automatic fallback to Access Point mode for configuration.
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include "config.h"

// WiFi mode enumeration
enum class WiFiMode {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    AP_MODE
};

// Network information structure
struct NetworkInfo {
    String ssid;
    int32_t rssi;
    uint8_t encryptionType;
    bool open;
};

/**
 * @class WiFiManager
 * @brief Manages WiFi connectivity with AP fallback
 */
class WiFiManager {
public:
    WiFiManager();
    
    /**
     * @brief Initialize WiFi manager
     * @return true if initialized
     */
    bool begin();
    
    /**
     * @brief Update WiFi state (call from main loop)
     */
    void update();
    
    /**
     * @brief Connect to a WiFi network
     * @param ssid Network SSID
     * @param password Network password
     * @param save Save credentials to NVS
     * @return true if connection started
     */
    bool connect(const char* ssid, const char* password, bool save = true);
    
    /**
     * @brief Disconnect from current network
     */
    void disconnect();
    
    /**
     * @brief Start Access Point mode
     * @return true if AP started
     */
    bool startAP();
    
    /**
     * @brief Stop Access Point mode
     */
    void stopAP();
    
    /**
     * @brief Get current WiFi mode
     */
    WiFiMode getMode() const { return _mode; }
    
    /**
     * @brief Check if connected to WiFi
     */
    bool isConnected() const { return _mode == WiFiMode::CONNECTED; }
    
    /**
     * @brief Check if in AP mode
     */
    bool isAPMode() const { return _mode == WiFiMode::AP_MODE; }
    
    /**
     * @brief Get current IP address
     */
    IPAddress getIP() const;
    
    /**
     * @brief Get current SSID
     */
    String getSSID() const;
    
    /**
     * @brief Get signal strength (RSSI)
     */
    int32_t getRSSI() const;
    
    /**
     * @brief Get hostname
     */
    String getHostname() const { return _hostname; }
    
    /**
     * @brief Scan for available networks
     * @param networks Array to fill with found networks
     * @param maxCount Maximum networks to return
     * @return Number of networks found
     */
    int scanNetworks(NetworkInfo* networks, int maxCount);
    
    /**
     * @brief Get saved SSID from NVS
     */
    String getSavedSSID();
    
    /**
     * @brief Clear saved credentials
     */
    void clearCredentials();
    
    /**
     * @brief Get status string
     */
    String getStatusString() const;
    
    /**
     * @brief Check if mDNS is active
     */
    bool isMDNSActive() const { return _mdnsActive; }

private:
    Preferences _prefs;
    WiFiMode _mode;
    String _hostname;
    bool _mdnsActive;
    
    // Connection state
    uint32_t _connectStartTime;
    bool _wasConnected;
    uint32_t _lastReconnectAttempt;
    
    // Saved credentials
    String _savedSSID;
    String _savedPassword;
    
    // Internal methods
    void loadCredentials();
    void saveCredentials(const char* ssid, const char* password);
    void setupMDNS();
    void handleConnectionLost();
};

// Global instance
extern WiFiManager wifiManager;

#endif // WIFI_MANAGER_H
