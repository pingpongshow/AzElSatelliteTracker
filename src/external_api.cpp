/**
 * @file external_api.cpp
 * @brief External API integrations implementation
 */

#include "external_api.h"
#include "utils.h"
#include "gps.h"
#include <WiFi.h>

// Global instance
ExternalAPI externalAPI;

// TLE Format Constants (matching tle_manager.cpp)
static const size_t TLE_LINE_MIN_LENGTH = 68;      // Minimum valid TLE line (without checksum)
static const char TLE_LINE1_ID = '1';              // Line 1 identifier
static const char TLE_LINE2_ID = '2';              // Line 2 identifier

// Known TLE groups on Celestrak
static const char* TLE_GROUPS[] = {
    "amateur", "stations", "weather", "noaa", "goes", 
    "resource", "sarsat", "dmc", "tdrss", "argos",
    "geo", "intelsat", "ses", "iridium", "iridium-NEXT",
    "starlink", "oneweb", "globalstar", "orbcomm",
    "gps-ops", "glonass", "galileo", "beidou",
    "sbas", "nnss", "musson", "science", "geodetic",
    "engineering", "education", "military", "radar",
    "cubesat", "other-comm", "satnogs", "spire"
};
static const int NUM_TLE_GROUPS = sizeof(TLE_GROUPS) / sizeof(TLE_GROUPS[0]);

ExternalAPI::ExternalAPI() 
    : _lastRequestTime(0) {
    // Enable HTTP connection reuse (keep-alive)
    _http.setReuse(true);
}

void ExternalAPI::setN2YOApiKey(const char* apiKey) {
    _n2yoApiKey = apiKey;
    DEBUG_PRINTLN("ExternalAPI: N2YO API key set");
}

String ExternalAPI::extractHost(const char* url) {
    // Extract host from URL for connection reuse tracking
    String urlStr = url;
    int protoEnd = urlStr.indexOf("://");
    if (protoEnd < 0) return "";
    
    int hostStart = protoEnd + 3;
    int hostEnd = urlStr.indexOf('/', hostStart);
    if (hostEnd < 0) hostEnd = urlStr.length();
    
    return urlStr.substring(hostStart, hostEnd);
}

bool ExternalAPI::httpGet(const char* url, String& response) {
    if (WiFi.status() != WL_CONNECTED) {
        SET_ERROR_RETURN("No network connection");
    }
    
    // Connection reuse timeout (30 seconds)
    const uint32_t CONNECTION_REUSE_TIMEOUT_MS = 30000;
    
    String currentHost = extractHost(url);
    uint32_t now = millis();
    
    // Check if we should close existing connection (different host or stale)
    bool shouldClose = false;
    if (_lastHost.length() > 0 && _lastHost != currentHost) {
        shouldClose = true;
    } else if (_lastRequestTime > 0 && (now - _lastRequestTime) > CONNECTION_REUSE_TIMEOUT_MS) {
        shouldClose = true;
    }
    
    if (shouldClose) {
        _http.end();
    }
    
    _http.setTimeout(N2YO_API_TIMEOUT_MS);
    
    if (!_http.begin(url)) {
        SET_ERROR_RETURN("HTTP begin failed");
    }
    
    int httpCode = _http.GET();
    
    // Update connection tracking
    _lastHost = currentHost;
    _lastRequestTime = millis();
    
    if (httpCode != HTTP_CODE_OK) {
        _lastError = "HTTP error: " + String(httpCode);
        _http.end();
        _lastHost = "";
        return false;
    }
    
    response = _http.getString();
    
    // Don't call _http.end() here - keep connection alive for reuse
    
    // Check for empty response
    if (response.length() == 0) {
        SET_ERROR_RETURN("Empty response from server");
    }
    
    return true;
}

// urlEncode moved to utils.h as shared utility

// N2YO Implementation

bool ExternalAPI::fetchTLEFromN2YO(uint32_t noradId, TLEEntry& entry) {
    if (!hasN2YOApiKey()) {
        SET_ERROR_RETURN("N2YO API key not set");
    }
    
    char url[256];
    snprintf(url, sizeof(url), 
             "%s/tle/%lu&apiKey=%s",
             N2YO_API_URL, noradId, _n2yoApiKey.c_str());
    
    String response;
    if (!httpGet(url, response)) {
        return false;  // _lastError already set by httpGet
    }
    
    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, response);
    if (error) {
        SET_ERROR_RETURN("JSON parse error");
    }
    
    if (doc.containsKey("error")) {
        _lastError = doc["error"].as<String>();
        return false;
    }
    
    const char* name = doc["info"]["satname"] | "";
    const char* tle = doc["tle"] | "";
    
    strncpy(entry.name, name, sizeof(entry.name) - 1);
    entry.name[sizeof(entry.name) - 1] = '\0';  // Ensure null termination
    
    // Parse TLE lines from combined string
    String tleStr = String(tle);
    int nlPos = tleStr.indexOf('\n');
    if (nlPos < 0) nlPos = tleStr.indexOf('\r');
    
    if (nlPos > 0) {
        strncpy(entry.line1, tleStr.substring(0, nlPos).c_str(), sizeof(entry.line1) - 1);
        entry.line1[sizeof(entry.line1) - 1] = '\0';  // Ensure null termination
        strncpy(entry.line2, tleStr.substring(nlPos + 1).c_str(), sizeof(entry.line2) - 1);
        entry.line2[sizeof(entry.line2) - 1] = '\0';  // Ensure null termination
    }
    
    entry.fetchTime = gps.getUnixTime();  // Use GPS Unix time (consistent with tle_manager)
    entry.valid = true;
    
    return true;
}

int ExternalAPI::getVisualPasses(uint32_t noradId, float lat, float lon, float alt,
                                  int days, int minVisibility,
                                  VisualPass* passes, int maxPasses) {
    if (!hasN2YOApiKey()) {
        SET_ERROR_RETURN_VAL("N2YO API key not set", 0);
    }
    
    char url[512];
    snprintf(url, sizeof(url),
             "%s/visualpasses/%lu/%.4f/%.4f/%.0f/%d/%d&apiKey=%s",
             N2YO_API_URL, noradId, lat, lon, alt, days, minVisibility, 
             _n2yoApiKey.c_str());
    
    String response;
    if (!httpGet(url, response)) {
        return 0;  // _lastError already set by httpGet
    }
    
    StaticJsonDocument<2048> doc;
    if (deserializeJson(doc, response)) {
        SET_ERROR_RETURN_VAL("JSON parse error", 0);
    }
    
    JsonArray passArray = doc["passes"].as<JsonArray>();
    int count = 0;
    
    for (JsonObject p : passArray) {
        if (count >= maxPasses) break;
        
        passes[count].startTime = p["startUTC"] | 0;
        passes[count].maxTime = p["maxUTC"] | 0;
        passes[count].endTime = p["endUTC"] | 0;
        passes[count].startAz = p["startAz"] | 0;
        passes[count].startEl = p["startEl"] | 0;
        passes[count].maxAz = p["maxAz"] | 0;
        passes[count].maxEl = p["maxEl"] | 0;
        passes[count].endAz = p["endAz"] | 0;
        passes[count].endEl = p["endEl"] | 0;
        passes[count].magnitude = p["mag"] | 99;
        passes[count].duration = p["duration"] | 0;
        
        count++;
    }
    
    return count;
}

int ExternalAPI::getRadioPasses(uint32_t noradId, float lat, float lon, float alt,
                                 int days, float minEl,
                                 PassInfo* passes, int maxPasses) {
    if (!hasN2YOApiKey()) {
        SET_ERROR_RETURN_VAL("N2YO API key not set", 0);
    }
    
    char url[512];
    snprintf(url, sizeof(url),
             "%s/radiopasses/%lu/%.4f/%.4f/%.0f/%d/%.0f&apiKey=%s",
             N2YO_API_URL, noradId, lat, lon, alt, days, minEl,
             _n2yoApiKey.c_str());
    
    String response;
    if (!httpGet(url, response)) {
        return 0;  // _lastError already set by httpGet
    }
    
    StaticJsonDocument<2048> doc;
    if (deserializeJson(doc, response)) {
        SET_ERROR_RETURN_VAL("JSON parse error", 0);
    }
    
    JsonArray passArray = doc["passes"].as<JsonArray>();
    int count = 0;
    
    for (JsonObject p : passArray) {
        if (count >= maxPasses) break;
        
        passes[count].aosTime = p["startUTC"] | 0;
        passes[count].maxElTime = p["maxUTC"] | 0;
        passes[count].losTime = p["endUTC"] | 0;
        passes[count].aosAz = p["startAz"] | 0;
        passes[count].maxElAz = p["maxAz"] | 0;
        passes[count].losAz = p["endAz"] | 0;
        passes[count].maxEl = p["maxEl"] | 0;
        
        count++;
    }
    
    return count;
}

int ExternalAPI::searchSatellites(const char* name, uint32_t* results, int maxResults) {
    // Use Celestrak GP search endpoint
    char url[256];
    snprintf(url, sizeof(url), "%s?NAME=%s&FORMAT=JSON", 
             CELESTRAK_API_URL, urlEncode(name).c_str());
    
    String response;
    if (!httpGet(url, response)) {
        return 0;  // _lastError already set by httpGet
    }
    
    StaticJsonDocument<2048> doc;
    if (deserializeJson(doc, response)) {
        SET_ERROR_RETURN_VAL("JSON parse error", 0);
    }
    
    // Celestrak returns array of objects with NORAD_CAT_ID
    JsonArray satArray = doc.as<JsonArray>();
    int count = 0;
    
    for (JsonObject sat : satArray) {
        if (count >= maxResults) break;
        results[count++] = sat["NORAD_CAT_ID"] | 0;
    }
    
    return count;
}

// SatNOGS Implementation

bool ExternalAPI::getSatelliteInfo(uint32_t noradId, SatNOGSSatellite& info) {
    char url[256];
    snprintf(url, sizeof(url), "%s/satellites/%lu/", SATNOGS_API_URL, noradId);
    
    String response;
    if (!httpGet(url, response)) {
        return false;  // _lastError already set by httpGet
    }
    
    StaticJsonDocument<2048> doc;
    if (deserializeJson(doc, response)) {
        SET_ERROR_RETURN("JSON parse error");
    }
    
    info.noradId = doc["norad_cat_id"] | noradId;
    strncpy(info.name, doc["name"] | "", sizeof(info.name) - 1);
    strncpy(info.altName, doc["names"] | "", sizeof(info.altName) - 1);
    strncpy(info.status, doc["status"] | "unknown", sizeof(info.status) - 1);
    
    // Check for transmitters
    info.hasTransmitter = false;
    info.downlinkMHz = 0;
    info.uplinkMHz = 0;
    
    return true;
}

int ExternalAPI::getTransmitters(uint32_t noradId, SatNOGSTransmitter* transmitters, int maxTx) {
    char url[256];
    snprintf(url, sizeof(url), "%s/transmitters/?satellite__norad_cat_id=%lu", 
             SATNOGS_API_URL, noradId);
    
    String response;
    if (!httpGet(url, response)) {
        return 0;  // _lastError already set by httpGet
    }
    
    StaticJsonDocument<2048> doc;
    if (deserializeJson(doc, response)) {
        SET_ERROR_RETURN_VAL("JSON parse error", 0);
    }
    
    JsonArray txArray = doc.as<JsonArray>();
    int count = 0;
    
    for (JsonObject tx : txArray) {
        if (count >= maxTx) break;
        
        transmitters[count].noradId = noradId;
        strncpy(transmitters[count].description, tx["description"] | "", 
                sizeof(transmitters[count].description) - 1);
        transmitters[count].downlinkLow = (tx["downlink_low"] | 0) / 1e6;
        transmitters[count].downlinkHigh = (tx["downlink_high"] | 0) / 1e6;
        transmitters[count].uplinkLow = (tx["uplink_low"] | 0) / 1e6;
        transmitters[count].uplinkHigh = (tx["uplink_high"] | 0) / 1e6;
        strncpy(transmitters[count].mode, tx["mode"] | "", sizeof(transmitters[count].mode) - 1);
        transmitters[count].alive = tx["alive"] | false;
        transmitters[count].baud = tx["baud"] | 0;
        
        count++;
    }
    
    return count;
}

int ExternalAPI::searchSatNOGS(const char* query, SatNOGSSatellite* results, int maxResults) {
    char url[256];
    snprintf(url, sizeof(url), "%s/satellites/?search=%s", 
             SATNOGS_API_URL, urlEncode(query).c_str());
    
    String response;
    if (!httpGet(url, response)) {
        return 0;  // _lastError already set by httpGet
    }
    
    StaticJsonDocument<2048> doc;
    if (deserializeJson(doc, response)) {
        SET_ERROR_RETURN_VAL("JSON parse error", 0);
    }
    
    JsonArray satArray = doc.as<JsonArray>();
    int count = 0;
    
    for (JsonObject sat : satArray) {
        if (count >= maxResults) break;
        
        results[count].noradId = sat["norad_cat_id"] | 0;
        strncpy(results[count].name, sat["name"] | "", sizeof(results[count].name) - 1);
        strncpy(results[count].status, sat["status"] | "", sizeof(results[count].status) - 1);
        
        count++;
    }
    
    return count;
}

// Celestrak Implementation

bool ExternalAPI::fetchTLEFromCelestrak(uint32_t noradId, TLEEntry& entry) {
    char url[256];
    snprintf(url, sizeof(url), "%s?CATNR=%lu&FORMAT=TLE", CELESTRAK_API_URL, noradId);
    
    String response;
    if (!httpGet(url, response)) {
        return false;  // _lastError already set by httpGet
    }
    
    // Parse 3-line TLE format
    int lineCount = 0;
    int pos = 0;
    String lines[3];
    
    while (pos < response.length() && lineCount < 3) {
        int nlPos = response.indexOf('\n', pos);
        if (nlPos < 0) nlPos = response.length();
        
        String line = response.substring(pos, nlPos);
        line.trim();
        
        if (line.length() > 0) {
            lines[lineCount++] = line;
        }
        
        pos = nlPos + 1;
    }
    
    if (lineCount < 3) {
        SET_ERROR_RETURN("Invalid TLE response");
    }
    
    strncpy(entry.name, lines[0].c_str(), sizeof(entry.name) - 1);
    entry.name[sizeof(entry.name) - 1] = '\0';
    strncpy(entry.line1, lines[1].c_str(), sizeof(entry.line1) - 1);
    entry.line1[sizeof(entry.line1) - 1] = '\0';
    strncpy(entry.line2, lines[2].c_str(), sizeof(entry.line2) - 1);
    entry.line2[sizeof(entry.line2) - 1] = '\0';
    entry.fetchTime = gps.getUnixTime();  // Use GPS Unix time (consistent with tle_manager)
    entry.valid = true;
    
    return true;
}

bool ExternalAPI::fetchTLEByName(const char* name, TLEEntry& entry) {
    char url[256];
    snprintf(url, sizeof(url), "%s?NAME=%s&FORMAT=TLE", 
             CELESTRAK_API_URL, urlEncode(name).c_str());
    
    String response;
    if (!httpGet(url, response)) {
        return false;  // _lastError already set by httpGet
    }
    
    // Parse response same as by ID
    int lineCount = 0;
    int pos = 0;
    String lines[3];
    
    while (pos < response.length() && lineCount < 3) {
        int nlPos = response.indexOf('\n', pos);
        if (nlPos < 0) nlPos = response.length();
        
        String line = response.substring(pos, nlPos);
        line.trim();
        
        if (line.length() > 0) {
            lines[lineCount++] = line;
        }
        
        pos = nlPos + 1;
    }
    
    if (lineCount < 3) {
        SET_ERROR_RETURN("Satellite not found");
    }
    
    strncpy(entry.name, lines[0].c_str(), sizeof(entry.name) - 1);
    entry.name[sizeof(entry.name) - 1] = '\0';
    strncpy(entry.line1, lines[1].c_str(), sizeof(entry.line1) - 1);
    entry.line1[sizeof(entry.line1) - 1] = '\0';
    strncpy(entry.line2, lines[2].c_str(), sizeof(entry.line2) - 1);
    entry.line2[sizeof(entry.line2) - 1] = '\0';
    entry.fetchTime = gps.getUnixTime();  // Use GPS Unix time (consistent with tle_manager)
    entry.valid = true;
    
    return true;
}

int ExternalAPI::fetchTLEGroup(const char* group, void (*callback)(const TLEEntry&)) {
    char url[256];
    snprintf(url, sizeof(url), "%s?GROUP=%s&FORMAT=TLE", CELESTRAK_API_URL, group);
    
    if (WiFi.status() != WL_CONNECTED) {
        SET_ERROR_RETURN_VAL("No network connection", 0);
    }
    
    // Connection reuse handling
    const uint32_t CONNECTION_REUSE_TIMEOUT_MS = 30000;
    String currentHost = extractHost(url);
    uint32_t now = millis();
    
    // For streaming requests, close any existing connection first
    // (streaming mode is incompatible with connection reuse mid-stream)
    if (_lastHost.length() > 0 && 
        (_lastHost != currentHost || (now - _lastRequestTime) > CONNECTION_REUSE_TIMEOUT_MS)) {
        _http.end();
    }
    
    _http.setTimeout(N2YO_API_TIMEOUT_MS);
    
    if (!_http.begin(url)) {
        SET_ERROR_RETURN_VAL("HTTP begin failed", 0);
    }
    
    int httpCode = _http.GET();
    
    // Update connection tracking
    _lastHost = currentHost;
    _lastRequestTime = millis();
    
    if (httpCode != HTTP_CODE_OK) {
        _lastError = "HTTP error: " + String(httpCode);
        _http.end();
        _lastHost = "";
        return 0;
    }
    
    // Stream-parse the response to avoid loading entire file into memory
    // This is critical for large TLE groups like Starlink (6000+ satellites)
    WiFiClient* stream = _http.getStreamPtr();
    if (!stream) {
        _lastError = "Failed to get stream";
        _http.end();
        return 0;
    }
    
    int count = 0;
    String lines[3];
    int lineCount = 0;
    String currentLine;
    currentLine.reserve(80);  // TLE lines are max 69 chars + some margin
    
    // Timeout handling to prevent hanging on stalled connections
    const uint32_t STREAM_TIMEOUT_MS = 60000;  // 60 second overall timeout
    const uint32_t IDLE_TIMEOUT_MS = 5000;     // 5 second idle timeout (no data received)
    uint32_t startTime = millis();
    uint32_t lastDataTime = millis();
    
    while (stream->available() || stream->connected()) {
        // Check overall timeout
        if (millis() - startTime > STREAM_TIMEOUT_MS) {
            DEBUG_PRINTLN("ExternalAPI: Stream overall timeout");
            break;
        }
        
        if (stream->available()) {
            lastDataTime = millis();  // Reset idle timer on data received
            char c = stream->read();
            
            if (c == '\n' || c == '\r') {
                // End of line
                currentLine.trim();
                if (currentLine.length() > 0) {
                    lines[lineCount++] = currentLine;
                    
                    // Process when we have 3 lines
                    if (lineCount >= 3) {
                        // Validate and create entry - accept lines with TLE_LINE_MIN_LENGTH+ chars
                        // (some sources trim trailing whitespace)
                        size_t len1 = lines[1].length();
                        size_t len2 = lines[2].length();
                        if (len1 >= TLE_LINE_MIN_LENGTH && len2 >= TLE_LINE_MIN_LENGTH &&
                            lines[1][0] == TLE_LINE1_ID && lines[2][0] == TLE_LINE2_ID) {
                            TLEEntry entry;
                            strncpy(entry.name, lines[0].c_str(), sizeof(entry.name) - 1);
                            entry.name[sizeof(entry.name) - 1] = '\0';
                            strncpy(entry.line1, lines[1].c_str(), sizeof(entry.line1) - 1);
                            entry.line1[sizeof(entry.line1) - 1] = '\0';
                            strncpy(entry.line2, lines[2].c_str(), sizeof(entry.line2) - 1);
                            entry.line2[sizeof(entry.line2) - 1] = '\0';
                            entry.fetchTime = gps.getUnixTime();  // Use GPS Unix time (consistent with tle_manager)
                            entry.valid = true;
                            
                            if (callback) {
                                callback(entry);
                            }
                            count++;
                        }
                        lineCount = 0;
                    }
                }
                currentLine = "";
            } else {
                currentLine += c;
            }
        } else {
            // Check idle timeout (no data but still connected)
            if (millis() - lastDataTime > IDLE_TIMEOUT_MS) {
                DEBUG_PRINTLN("ExternalAPI: Stream idle timeout");
                break;
            }
            // Wait for more data with small delay
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        
        // Yield periodically to avoid watchdog timeout
        if (count % 100 == 0) {
            vTaskDelay(1);  // Use vTaskDelay for proper FreeRTOS yielding
        }
    }
    
    // Handle any remaining data (in case file doesn't end with newline)
    currentLine.trim();
    if (currentLine.length() > 0 && lineCount < 3) {
        lines[lineCount++] = currentLine;
        
        if (lineCount >= 3) {
            // Accept lines with TLE_LINE_MIN_LENGTH+ chars (some sources trim trailing whitespace)
            size_t len1 = lines[1].length();
            size_t len2 = lines[2].length();
            if (len1 >= TLE_LINE_MIN_LENGTH && len2 >= TLE_LINE_MIN_LENGTH &&
                lines[1][0] == TLE_LINE1_ID && lines[2][0] == TLE_LINE2_ID) {
                TLEEntry entry;
                strncpy(entry.name, lines[0].c_str(), sizeof(entry.name) - 1);
                entry.name[sizeof(entry.name) - 1] = '\0';
                strncpy(entry.line1, lines[1].c_str(), sizeof(entry.line1) - 1);
                entry.line1[sizeof(entry.line1) - 1] = '\0';
                strncpy(entry.line2, lines[2].c_str(), sizeof(entry.line2) - 1);
                entry.line2[sizeof(entry.line2) - 1] = '\0';
                entry.fetchTime = gps.getUnixTime();  // Use GPS Unix time (consistent with tle_manager)
                entry.valid = true;
                
                if (callback) {
                    callback(entry);
                }
                count++;
            }
        }
    }
    
    _http.end();
    
    // Update request time after streaming completes
    _lastRequestTime = millis();
    
    return count;
}

int ExternalAPI::getTLEGroups(String* groups, int maxGroups) {
    int count = min(maxGroups, NUM_TLE_GROUPS);
    for (int i = 0; i < count; i++) {
        groups[i] = TLE_GROUPS[i];
    }
    return count;
}
