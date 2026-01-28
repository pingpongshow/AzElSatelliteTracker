/**
 * @file wifi_manager.cpp
 * @brief WiFi connection management implementation
 */

#include "wifi_manager.h"

// Global instance
WiFiManager wifiManager;

WiFiManager::WiFiManager()
    : _mode(WiFiMode::DISCONNECTED)
    , _hostname(MDNS_HOSTNAME)
    , _mdnsActive(false)
    , _connectStartTime(0)
    , _wasConnected(false)
    , _lastReconnectAttempt(0)
{
}

bool WiFiManager::begin() {
    DEBUG_PRINTLN("WiFiManager: Initializing...");
    
    // Open preferences
    _prefs.begin(NVS_NAMESPACE, false);
    
    // Load saved credentials
    loadCredentials();
    
    // Set hostname
    WiFi.setHostname(_hostname.c_str());
    
    // Try to connect with saved credentials
    if (_savedSSID.length() > 0) {
        DEBUG_PRINTF("WiFiManager: Found saved network '%s'\n", _savedSSID.c_str());
        connect(_savedSSID.c_str(), _savedPassword.c_str(), false);
    } else {
        // No saved credentials, start AP
        DEBUG_PRINTLN("WiFiManager: No saved credentials, starting AP");
        startAP();
    }
    
    return true;
}

void WiFiManager::update() {
    uint32_t now = millis();
    
    switch (_mode) {
        case WiFiMode::CONNECTING:
            // Check connection timeout
            if (now - _connectStartTime > WIFI_CONNECT_TIMEOUT_MS) {
                DEBUG_PRINTLN("WiFiManager: Connection timeout");
                WiFi.disconnect();
                startAP();
            } else if (WiFi.status() == WL_CONNECTED) {
                _mode = WiFiMode::CONNECTED;
                _wasConnected = true;
                DEBUG_PRINTF("WiFiManager: Connected! IP: %s\n", 
                             WiFi.localIP().toString().c_str());
                setupMDNS();
            }
            break;
            
        case WiFiMode::CONNECTED:
            if (WiFi.status() != WL_CONNECTED) {
                handleConnectionLost();
            }
            break;
            
        case WiFiMode::DISCONNECTED:
            // Try reconnect if we have credentials
            if (_savedSSID.length() > 0 && 
                now - _lastReconnectAttempt > WIFI_RECONNECT_INTERVAL_MS) {
                DEBUG_PRINTLN("WiFiManager: Attempting reconnect...");
                connect(_savedSSID.c_str(), _savedPassword.c_str(), false);
                _lastReconnectAttempt = now;
            }
            break;
            
        case WiFiMode::AP_MODE:
            // Nothing to do in AP mode
            break;
    }
}

bool WiFiManager::connect(const char* ssid, const char* password, bool save) {
    DEBUG_PRINTF("WiFiManager: Connecting to '%s'...\n", ssid);
    
    // Stop any existing connection
    WiFi.disconnect(true);
    delay(100);
    
    // Set mode to station
    WiFi.mode(WIFI_STA);
    
    // Start connection
    WiFi.begin(ssid, password);
    
    _mode = WiFiMode::CONNECTING;
    _connectStartTime = millis();
    
    // Save credentials if requested
    if (save) {
        saveCredentials(ssid, password);
    }
    
    return true;
}

void WiFiManager::disconnect() {
    WiFi.disconnect(true);
    _mode = WiFiMode::DISCONNECTED;
    _mdnsActive = false;
    DEBUG_PRINTLN("WiFiManager: Disconnected");
}

bool WiFiManager::startAP() {
    DEBUG_PRINTF("WiFiManager: Starting AP '%s'...\n", AP_SSID);
    
    // Fully reset WiFi
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);
    
    // Configure and start AP
    WiFi.mode(WIFI_AP);
    delay(100);
    
    // Configure AP with explicit settings
    // softAP(ssid, password, channel, hidden, max_connections)
    bool result = WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, false, AP_MAX_CONNECTIONS);
    
    if (!result) {
        DEBUG_PRINTLN("WiFiManager: ERROR - softAP() failed!");
        return false;
    }
    
    // Wait for AP to fully start
    delay(500);
    
    // Verify AP is running
    IPAddress apIP = WiFi.softAPIP();
    if (apIP == IPAddress(0, 0, 0, 0)) {
        DEBUG_PRINTLN("WiFiManager: ERROR - AP IP is 0.0.0.0!");
        return false;
    }
    
    _mode = WiFiMode::AP_MODE;
    
    DEBUG_PRINTF("WiFiManager: AP started successfully!\n");
    DEBUG_PRINTF("WiFiManager:   SSID: %s\n", AP_SSID);
    DEBUG_PRINTF("WiFiManager:   Password: %s\n", AP_PASSWORD);
    DEBUG_PRINTF("WiFiManager:   Channel: %d\n", AP_CHANNEL);
    DEBUG_PRINTF("WiFiManager:   IP: %s\n", apIP.toString().c_str());
    DEBUG_PRINTF("WiFiManager:   MAC: %s\n", WiFi.softAPmacAddress().c_str());
    
    // Setup mDNS for AP mode too
    setupMDNS();
    
    return true;
}

void WiFiManager::stopAP() {
    if (_mode == WiFiMode::AP_MODE) {
        WiFi.softAPdisconnect(true);
        _mode = WiFiMode::DISCONNECTED;
        DEBUG_PRINTLN("WiFiManager: AP stopped");
    }
}

IPAddress WiFiManager::getIP() const {
    if (_mode == WiFiMode::AP_MODE) {
        return WiFi.softAPIP();
    }
    return WiFi.localIP();
}

String WiFiManager::getSSID() const {
    if (_mode == WiFiMode::AP_MODE) {
        return AP_SSID;
    }
    return WiFi.SSID();
}

int32_t WiFiManager::getRSSI() const {
    if (_mode == WiFiMode::CONNECTED) {
        return WiFi.RSSI();
    }
    return 0;
}

int WiFiManager::scanNetworks(NetworkInfo* networks, int maxCount) {
    DEBUG_PRINTLN("WiFiManager: Scanning networks...");
    
    int found = WiFi.scanNetworks();
    
    if (found < 0) {
        DEBUG_PRINTLN("WiFiManager: Scan failed");
        return 0;
    }
    
    int count = min(found, maxCount);
    
    for (int i = 0; i < count; i++) {
        networks[i].ssid = WiFi.SSID(i);
        networks[i].rssi = WiFi.RSSI(i);
        networks[i].encryptionType = WiFi.encryptionType(i);
        networks[i].open = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
    }
    
    WiFi.scanDelete();
    
    DEBUG_PRINTF("WiFiManager: Found %d networks\n", count);
    return count;
}

String WiFiManager::getSavedSSID() {
    return _savedSSID;
}

void WiFiManager::clearCredentials() {
    _prefs.remove(NVS_KEY_WIFI_SSID);
    _prefs.remove(NVS_KEY_WIFI_PASS);
    _savedSSID = "";
    _savedPassword = "";
    DEBUG_PRINTLN("WiFiManager: Credentials cleared");
}

String WiFiManager::getStatusString() const {
    String status = "WiFi: ";
    
    switch (_mode) {
        case WiFiMode::DISCONNECTED:
            status += "Disconnected";
            break;
        case WiFiMode::CONNECTING:
            status += "Connecting to " + _savedSSID;
            break;
        case WiFiMode::CONNECTED:
            status += "Connected to " + WiFi.SSID();
            status += " (" + String(WiFi.RSSI()) + " dBm)";
            status += " IP: " + WiFi.localIP().toString();
            break;
        case WiFiMode::AP_MODE:
            status += "AP Mode: " + String(AP_SSID);
            status += " IP: " + WiFi.softAPIP().toString();
            status += " Clients: " + String(WiFi.softAPgetStationNum());
            break;
    }
    
    if (_mdnsActive) {
        status += " [" + _hostname + ".local]";
    }
    
    return status;
}

void WiFiManager::loadCredentials() {
    _savedSSID = _prefs.getString(NVS_KEY_WIFI_SSID, "");
    _savedPassword = _prefs.getString(NVS_KEY_WIFI_PASS, "");
}

void WiFiManager::saveCredentials(const char* ssid, const char* password) {
    _prefs.putString(NVS_KEY_WIFI_SSID, ssid);
    _prefs.putString(NVS_KEY_WIFI_PASS, password);
    _savedSSID = ssid;
    _savedPassword = password;
    DEBUG_PRINTF("WiFiManager: Credentials saved for '%s'\n", ssid);
}

void WiFiManager::setupMDNS() {
    if (MDNS.begin(_hostname.c_str())) {
        _mdnsActive = true;
        
        // Advertise services
        MDNS.addService("http", "tcp", WEB_SERVER_PORT);
        MDNS.addService("rotctld", "tcp", ROTCTLD_PORT);
        
        DEBUG_PRINTF("WiFiManager: mDNS started as %s.local\n", _hostname.c_str());
    } else {
        DEBUG_PRINTLN("WiFiManager: mDNS failed to start");
    }
}

void WiFiManager::handleConnectionLost() {
    DEBUG_PRINTLN("WiFiManager: Connection lost");
    _mode = WiFiMode::DISCONNECTED;
    _mdnsActive = false;
    _lastReconnectAttempt = millis();
}
