/**
 * @file tle_manager.cpp
 * @brief TLE fetching and storage implementation
 */

#include "tle_manager.h"
#include "gps.h"
#include "utils.h"
#include <WiFi.h>

// TLE Format Constants
// Standard TLE lines are 69 characters (68 data + 1 checksum)
// Some sources omit the checksum, giving 68 characters
static const size_t TLE_LINE_MIN_LENGTH = 68;      // Minimum valid TLE line (without checksum)
static const size_t TLE_LINE_FULL_LENGTH = 69;     // Full TLE line (with checksum)
static const size_t TLE_CHECKSUM_POS = 68;         // Position of checksum digit (0-indexed)
static const size_t TLE_CHECKSUM_DATA_LEN = 68;    // Number of characters used for checksum calculation
static const char TLE_LINE1_ID = '1';              // Line 1 identifier
static const char TLE_LINE2_ID = '2';              // Line 2 identifier

// Global instance
TLEManager tleManager;

TLEManager::TLEManager() 
    : _lastRequestTime(0)
    , _slotBitmap(0)
    , _slotBitmapInitialized(false) {
}

bool TLEManager::begin() {
    DEBUG_PRINTLN("TLEManager: Initializing...");
    
    // Enable HTTP connection reuse (keep-alive)
    _http.setReuse(true);
    
    // Open preferences namespace
    if (!_prefs.begin("tle_storage", false)) {
        DEBUG_PRINTLN("TLEManager: Failed to open preferences");
        return false;
    }
    
    // Initialize slot bitmap from storage
    initSlotBitmap();
    
    DEBUG_PRINTF("TLEManager: %d TLEs in storage\n", getStoredCount());
    return true;
}

bool TLEManager::fetchByNoradId(uint32_t noradId, TLEEntry& entry) {
    if (!isNetworkAvailable()) {
        SET_ERROR_RETURN("No network connection");
    }
    
    // Build Celestrak GP URL for single satellite
    char url[128];
    snprintf(url, sizeof(url), 
             "https://celestrak.org/NORAD/elements/gp.php?CATNR=%lu&FORMAT=TLE",
             noradId);
    
    DEBUG_PRINTF("TLEManager: Fetching TLE for NORAD %lu\n", noradId);
    
    String response;
    if (!httpFetch(url, response)) {
        return false;  // _lastError already set by httpFetch
    }
    
    // Parse the response
    if (!parseTLE(response.c_str(), entry)) {
        SET_ERROR_RETURN("Failed to parse TLE response");
    }
    
    entry.fetchTime = gps.getUnixTime();  // Use GPS Unix time (won't wrap like millis)
    entry.valid = true;
    
    DEBUG_PRINTF("TLEManager: Fetched TLE for %s\n", entry.name);
    return true;
}

bool TLEManager::fetchByName(const char* name, TLEEntry& entry) {
    if (!isNetworkAvailable()) {
        SET_ERROR_RETURN("No network connection");
    }
    
    // URL encode the name using shared utility
    String encodedName = urlEncode(name);
    
    // Build URL
    String url = String(TLE_SOURCE_CELESTRAK) + "?NAME=" + encodedName + "&FORMAT=TLE";
    
    DEBUG_PRINTF("TLEManager: Fetching TLE for '%s'\n", name);
    
    String response;
    if (!httpFetch(url.c_str(), response)) {
        return false;  // _lastError already set by httpFetch
    }
    
    // Parse the response
    if (!parseTLE(response.c_str(), entry)) {
        SET_ERROR_RETURN("Satellite not found or parse error");
    }
    
    entry.fetchTime = gps.getUnixTime();  // Use GPS Unix time (won't wrap like millis)
    entry.valid = true;
    
    DEBUG_PRINTF("TLEManager: Fetched TLE for %s\n", entry.name);
    return true;
}

int TLEManager::fetchAmateurSatellites(void (*callback)(const TLEEntry& entry)) {
    if (!isNetworkAvailable()) {
        SET_ERROR_RETURN_VAL("No network connection", 0);
    }
    
    DEBUG_PRINTLN("TLEManager: Fetching amateur satellite TLEs...");
    
    String response;
    if (!httpFetch(TLE_SOURCE_AMATEUR, response)) {
        return 0;  // _lastError already set by httpFetch
    }
    
    int count = 0;
    const char* ptr = response.c_str();
    
    while (*ptr) {
        TLEEntry entry;
        memset(&entry, 0, sizeof(entry));  // Initialize to zero for safety
        
        // Find next TLE (3 lines)
        const char* nameStart = ptr;
        const char* line1Start = strchr(nameStart, '\n');
        if (!line1Start) break;
        line1Start++;
        
        const char* line2Start = strchr(line1Start, '\n');
        if (!line2Start) break;
        line2Start++;
        
        const char* nextEntry = strchr(line2Start, '\n');
        
        // Validate pointer ordering before calculating lengths
        // This prevents undefined behavior from malformed TLE data
        if (line1Start <= nameStart || line2Start <= line1Start) {
            // Malformed data - skip this entry
            if (nextEntry) {
                ptr = nextEntry + 1;
                continue;
            } else {
                break;
            }
        }
        
        // Extract name - use size_t for safe pointer arithmetic
        size_t nameLen = (size_t)(line1Start - nameStart - 1);
        if (nameLen > 0 && nameLen < sizeof(entry.name)) {
            strncpy(entry.name, nameStart, nameLen);
            entry.name[nameLen] = '\0';
            
            // Trim trailing whitespace
            while (nameLen > 0 && entry.name[nameLen-1] == ' ') {
                entry.name[--nameLen] = '\0';
            }
        } else {
            // Invalid name length - skip entry
            if (nextEntry) {
                ptr = nextEntry + 1;
                continue;
            } else {
                break;
            }
        }
        
        // Extract line 1 - validate length before copy
        size_t line1Len = (size_t)(line2Start - line1Start - 1);
        if (line1Len >= TLE_LINE_FULL_LENGTH) {
            strncpy(entry.line1, line1Start, TLE_LINE_FULL_LENGTH);
            entry.line1[TLE_LINE_FULL_LENGTH] = '\0';
        } else {
            // Line 1 too short - skip entry
            if (nextEntry) {
                ptr = nextEntry + 1;
                continue;
            } else {
                break;
            }
        }
        
        // Extract line 2 - validate pointer and length
        size_t line2Len;
        if (nextEntry && nextEntry > line2Start) {
            line2Len = (size_t)(nextEntry - line2Start);
        } else {
            line2Len = strlen(line2Start);
        }
        
        if (line2Len >= TLE_LINE_FULL_LENGTH) {
            strncpy(entry.line2, line2Start, TLE_LINE_FULL_LENGTH);
            entry.line2[TLE_LINE_FULL_LENGTH] = '\0';
        } else {
            // Line 2 too short - skip entry
            if (nextEntry) {
                ptr = nextEntry + 1;
                continue;
            } else {
                break;
            }
        }
        
        // Validate and callback
        if (validateTLE(entry.line1, entry.line2)) {
            entry.fetchTime = gps.getUnixTime();  // Use GPS Unix time (won't wrap like millis)
            entry.valid = true;
            callback(entry);
            count++;
        }
        
        // Move to next entry
        if (nextEntry) {
            ptr = nextEntry + 1;
        } else {
            break;
        }
    }
    
    DEBUG_PRINTF("TLEManager: Fetched %d amateur satellite TLEs\n", count);
    return count;
}

bool TLEManager::saveTLE(const TLEEntry& entry, int slot) {
    if (slot < 0 || slot >= MAX_STORED_TLES) {
        return false;
    }
    
    char key[16];
    
    snprintf(key, sizeof(key), "tle_%d_name", slot);
    _prefs.putString(key, entry.name);
    
    snprintf(key, sizeof(key), "tle_%d_l1", slot);
    _prefs.putString(key, entry.line1);
    
    snprintf(key, sizeof(key), "tle_%d_l2", slot);
    _prefs.putString(key, entry.line2);
    
    snprintf(key, sizeof(key), "tle_%d_time", slot);
    _prefs.putUInt(key, entry.fetchTime);
    
    snprintf(key, sizeof(key), "tle_%d_valid", slot);
    _prefs.putBool(key, true);
    
    // Update slot bitmap
    markSlotOccupied(slot);
    
    DEBUG_PRINTF("TLEManager: Saved TLE '%s' to slot %d\n", entry.name, slot);
    return true;
}

bool TLEManager::loadTLE(int slot, TLEEntry& entry) {
    if (slot < 0 || slot >= MAX_STORED_TLES) {
        return false;
    }
    
    char key[16];
    
    snprintf(key, sizeof(key), "tle_%d_valid", slot);
    if (!_prefs.getBool(key, false)) {
        return false;
    }
    
    snprintf(key, sizeof(key), "tle_%d_name", slot);
    String name = _prefs.getString(key, "");
    strncpy(entry.name, name.c_str(), sizeof(entry.name) - 1);
    entry.name[sizeof(entry.name) - 1] = '\0';
    
    snprintf(key, sizeof(key), "tle_%d_l1", slot);
    String line1 = _prefs.getString(key, "");
    strncpy(entry.line1, line1.c_str(), sizeof(entry.line1) - 1);
    entry.line1[sizeof(entry.line1) - 1] = '\0';
    
    snprintf(key, sizeof(key), "tle_%d_l2", slot);
    String line2 = _prefs.getString(key, "");
    strncpy(entry.line2, line2.c_str(), sizeof(entry.line2) - 1);
    entry.line2[sizeof(entry.line2) - 1] = '\0';
    
    snprintf(key, sizeof(key), "tle_%d_time", slot);
    entry.fetchTime = _prefs.getUInt(key, 0);
    
    entry.valid = validateTLE(entry.line1, entry.line2);
    
    return entry.valid;
}

void TLEManager::deleteTLE(int slot) {
    if (slot < 0 || slot >= MAX_STORED_TLES) {
        return;
    }
    
    char key[16];
    snprintf(key, sizeof(key), "tle_%d_valid", slot);
    _prefs.putBool(key, false);
    
    // Update slot bitmap
    markSlotEmpty(slot);
    
    DEBUG_PRINTF("TLEManager: Deleted TLE from slot %d\n", slot);
}

bool TLEManager::deleteTLE(const char* name) {
    if (!name || !name[0]) {
        return false;
    }
    
    char key[16];
    
    // Find slot by name (case insensitive search)
    for (int i = 0; i < MAX_STORED_TLES; i++) {
        snprintf(key, sizeof(key), "tle_%d_valid", i);
        if (!_prefs.getBool(key, false)) continue;
        
        snprintf(key, sizeof(key), "tle_%d_name", i);
        String storedName = _prefs.getString(key, "");
        
        if (storedName.equalsIgnoreCase(name)) {
            deleteTLE(i);
            DEBUG_PRINTF("TLEManager: Deleted TLE '%s' from slot %d\n", name, i);
            return true;
        }
    }
    
    DEBUG_PRINTF("TLEManager: TLE '%s' not found for deletion\n", name);
    return false;
}

int TLEManager::getStoredCount() {
    int count = 0;
    char key[16];
    
    for (int i = 0; i < MAX_STORED_TLES; i++) {
        snprintf(key, sizeof(key), "tle_%d_valid", i);
        if (_prefs.getBool(key, false)) {
            count++;
        }
    }
    
    return count;
}

int TLEManager::listStoredTLEs(String* names, int maxCount) {
    int count = 0;
    char key[16];
    
    for (int i = 0; i < MAX_STORED_TLES && count < maxCount; i++) {
        snprintf(key, sizeof(key), "tle_%d_valid", i);
        if (_prefs.getBool(key, false)) {
            snprintf(key, sizeof(key), "tle_%d_name", i);
            names[count++] = _prefs.getString(key, "");
        }
    }
    
    return count;
}

bool TLEManager::findStoredTLE(const char* name, TLEEntry& entry) {
    char key[16];
    
    for (int i = 0; i < MAX_STORED_TLES; i++) {
        snprintf(key, sizeof(key), "tle_%d_valid", i);
        if (!_prefs.getBool(key, false)) continue;
        
        snprintf(key, sizeof(key), "tle_%d_name", i);
        String storedName = _prefs.getString(key, "");
        
        if (storedName.equalsIgnoreCase(name)) {
            return loadTLE(i, entry);
        }
    }
    
    return false;
}

bool TLEManager::parseTLE(const char* text, TLEEntry& entry) {
    // Initialize entry to zero for safety
    memset(&entry, 0, sizeof(entry));
    
    // Find the three lines
    const char* lines[3] = {nullptr, nullptr, nullptr};
    int lineCount = 0;
    
    const char* ptr = text;
    while (*ptr && lineCount < 3) {
        // Skip whitespace
        while (*ptr == ' ' || *ptr == '\t') ptr++;
        
        if (*ptr && *ptr != '\r' && *ptr != '\n') {
            lines[lineCount++] = ptr;
        }
        
        // Find end of line
        while (*ptr && *ptr != '\r' && *ptr != '\n') ptr++;
        
        // Skip line endings
        while (*ptr == '\r' || *ptr == '\n') ptr++;
    }
    
    if (lineCount < 3) {
        return false;
    }
    
    // Extract satellite name (line 0)
    const char* nameEnd = lines[0];
    while (*nameEnd && *nameEnd != '\r' && *nameEnd != '\n') nameEnd++;
    
    // Validate pointer ordering and calculate length safely
    if (nameEnd < lines[0]) {
        return false;  // Malformed data
    }
    size_t nameLen = (size_t)(nameEnd - lines[0]);
    if (nameLen >= sizeof(entry.name)) {
        nameLen = sizeof(entry.name) - 1;
    }
    if (nameLen == 0) {
        return false;  // Empty name
    }
    strncpy(entry.name, lines[0], nameLen);
    entry.name[nameLen] = '\0';
    
    // Trim trailing whitespace from name
    while (nameLen > 0 && entry.name[nameLen-1] == ' ') {
        entry.name[--nameLen] = '\0';
    }
    
    // Extract line 1
    const char* line1End = lines[1];
    while (*line1End && *line1End != '\r' && *line1End != '\n') line1End++;
    
    if (line1End <= lines[1]) {
        return false;  // Malformed data (empty line)
    }
    size_t line1Len = (size_t)(line1End - lines[1]);
    if (line1Len > TLE_LINE_FULL_LENGTH) line1Len = TLE_LINE_FULL_LENGTH;
    if (line1Len < TLE_LINE_MIN_LENGTH) {
        return false;  // TLE line 1 must be at least TLE_LINE_MIN_LENGTH characters
    }
    strncpy(entry.line1, lines[1], line1Len);
    entry.line1[line1Len] = '\0';
    
    // Extract line 2
    const char* line2End = lines[2];
    while (*line2End && *line2End != '\r' && *line2End != '\n') line2End++;
    
    if (line2End <= lines[2]) {
        return false;  // Malformed data (empty line)
    }
    size_t line2Len = (size_t)(line2End - lines[2]);
    if (line2Len > TLE_LINE_FULL_LENGTH) line2Len = TLE_LINE_FULL_LENGTH;
    if (line2Len < TLE_LINE_MIN_LENGTH) {
        return false;  // TLE line 2 must be at least TLE_LINE_MIN_LENGTH characters
    }
    strncpy(entry.line2, lines[2], line2Len);
    entry.line2[line2Len] = '\0';
    
    // Validate
    return validateTLE(entry.line1, entry.line2);
}

bool TLEManager::isTLEStale(const TLEEntry& entry) {
    // TLE is stale if older than 7 days
    // Use GPS Unix time to avoid millis() wrap-around after 49 days
    uint32_t now = gps.getUnixTime();
    
    // If we don't have valid time or fetch time, can't determine staleness
    if (now == 0 || entry.fetchTime == 0) {
        return false;  // Assume not stale if we can't check
    }
    
    return (now - entry.fetchTime) > (7 * 24 * 3600);
}

bool TLEManager::isNetworkAvailable() {
    return WiFi.status() == WL_CONNECTED;
}

String TLEManager::extractHost(const char* url) {
    // Extract host from URL for connection reuse tracking
    // Example: "https://celestrak.org/path" -> "celestrak.org"
    String urlStr = url;
    int protoEnd = urlStr.indexOf("://");
    if (protoEnd < 0) return "";
    
    int hostStart = protoEnd + 3;
    int hostEnd = urlStr.indexOf('/', hostStart);
    if (hostEnd < 0) hostEnd = urlStr.length();
    
    return urlStr.substring(hostStart, hostEnd);
}

bool TLEManager::httpFetch(const char* url, String& response) {
    // Connection reuse timeout (30 seconds)
    const uint32_t CONNECTION_REUSE_TIMEOUT_MS = 30000;
    
    String currentHost = extractHost(url);
    uint32_t now = millis();
    
    // Check if we should close existing connection (different host or stale)
    bool shouldClose = false;
    if (_lastHost.length() > 0 && _lastHost != currentHost) {
        // Different host - close existing connection
        shouldClose = true;
        DEBUG_PRINTF("TLEManager: Host changed (%s -> %s), closing connection\n", 
                     _lastHost.c_str(), currentHost.c_str());
    } else if (_lastRequestTime > 0 && (now - _lastRequestTime) > CONNECTION_REUSE_TIMEOUT_MS) {
        // Connection too old - close it
        shouldClose = true;
        DEBUG_PRINTLN("TLEManager: Connection stale, closing");
    }
    
    if (shouldClose) {
        _http.end();
    }
    
    _http.setTimeout(TLE_FETCH_TIMEOUT_MS);
    
    DEBUG_PRINTF("TLEManager: HTTP GET %s\n", url);
    
    if (!_http.begin(url)) {
        SET_ERROR_RETURN("Failed to begin HTTP connection");
    }
    
    int httpCode = _http.GET();
    
    // Update connection tracking
    _lastHost = currentHost;
    _lastRequestTime = millis();
    
    if (httpCode != HTTP_CODE_OK) {
        _lastError = "HTTP error: " + String(httpCode);
        _http.end();
        _lastHost = "";  // Clear host on error
        return false;
    }
    
    response = _http.getString();
    
    // Don't call _http.end() here - keep connection alive for reuse
    // Connection will be closed on host change, timeout, or next begin() call
    
    if (response.length() == 0) {
        SET_ERROR_RETURN("Empty response");
    }
    
    return true;
}

// =========================================================================
// Slot Bitmap Management for O(1) slot finding
// =========================================================================

void TLEManager::initSlotBitmap() {
    _slotBitmap = 0;
    
    // Scan all slots to initialize bitmap
    for (int i = 0; i < MAX_STORED_TLES && i < 32; i++) {
        char key[16];
        snprintf(key, sizeof(key), "tle_%d_valid", i);
        if (_prefs.getBool(key, false)) {
            _slotBitmap |= (1U << i);
        }
    }
    
    _slotBitmapInitialized = true;
    DEBUG_PRINTF("TLEManager: Slot bitmap initialized: 0x%08X\n", _slotBitmap);
}

int TLEManager::findEmptySlot() {
    if (!_slotBitmapInitialized) {
        initSlotBitmap();
    }
    
    // Find first zero bit (empty slot) using bit manipulation
    // ~_slotBitmap gives us 1s where slots are empty
    // __builtin_ffs finds first set bit (1-indexed), or 0 if none
    uint32_t emptyBits = ~_slotBitmap;
    
    // Mask to only consider valid slot indices
    uint32_t validMask = (MAX_STORED_TLES >= 32) ? 0xFFFFFFFF : ((1U << MAX_STORED_TLES) - 1);
    emptyBits &= validMask;
    
    if (emptyBits == 0) {
        return -1;  // All slots full
    }
    
    // __builtin_ctz counts trailing zeros (gives index of first set bit)
    return __builtin_ctz(emptyBits);
}

int TLEManager::findSlotByName(const char* name) {
    if (!name) return -1;
    
    // Still need to iterate for name matching, but only occupied slots
    if (!_slotBitmapInitialized) {
        initSlotBitmap();
    }
    
    uint32_t occupied = _slotBitmap;
    while (occupied) {
        // Get index of lowest set bit
        int slot = __builtin_ctz(occupied);
        
        // Check if this slot has matching name
        TLEEntry entry;
        if (loadTLE(slot, entry) && strcasecmp(entry.name, name) == 0) {
            return slot;
        }
        
        // Clear this bit and continue
        occupied &= ~(1U << slot);
    }
    
    return -1;  // Not found
}

void TLEManager::markSlotOccupied(int slot) {
    if (slot >= 0 && slot < 32 && slot < MAX_STORED_TLES) {
        _slotBitmap |= (1U << slot);
    }
}

void TLEManager::markSlotEmpty(int slot) {
    if (slot >= 0 && slot < 32 && slot < MAX_STORED_TLES) {
        _slotBitmap &= ~(1U << slot);
    }
}

bool TLEManager::validateTLE(const char* line1, const char* line2) {
    if (!line1 || !line2) {
        return false;
    }
    
    // Get effective length (excluding trailing whitespace)
    size_t len1 = strlen(line1);
    size_t len2 = strlen(line2);
    
    while (len1 > 0 && (line1[len1-1] == ' ' || line1[len1-1] == '\t' || 
                        line1[len1-1] == '\r' || line1[len1-1] == '\n')) {
        len1--;
    }
    while (len2 > 0 && (line2[len2-1] == ' ' || line2[len2-1] == '\t' || 
                        line2[len2-1] == '\r' || line2[len2-1] == '\n')) {
        len2--;
    }
    
    // TLE lines must be at least TLE_LINE_MIN_LENGTH characters
    // Accept TLE_LINE_MIN_LENGTH to TLE_LINE_FULL_LENGTH to handle sources 
    // that may omit or include the checksum digit
    if (len1 < TLE_LINE_MIN_LENGTH || len2 < TLE_LINE_MIN_LENGTH) {
        DEBUG_PRINTF("TLEManager: Line too short (L1: %d, L2: %d, min: %d)\n", 
                     (int)len1, (int)len2, (int)TLE_LINE_MIN_LENGTH);
        return false;
    }
    
    if (line1[0] != TLE_LINE1_ID || line2[0] != TLE_LINE2_ID) {
        DEBUG_PRINTLN("TLEManager: Invalid line identifiers");
        return false;
    }
    
    // Verify checksums only if lines have the checksum digit
    if (len1 >= TLE_LINE_FULL_LENGTH && len2 >= TLE_LINE_FULL_LENGTH) {
        int check1 = checksumTLELine(line1);
        int check2 = checksumTLELine(line2);
        
        int expected1 = line1[TLE_CHECKSUM_POS] - '0';
        int expected2 = line2[TLE_CHECKSUM_POS] - '0';
        
        if (check1 != expected1 || check2 != expected2) {
            DEBUG_PRINTF("TLEManager: Checksum mismatch (L1: %d!=%d, L2: %d!=%d)\n",
                         check1, expected1, check2, expected2);
            // Some TLEs have bad checksums, so just warn but don't fail
        }
    }
    
    return true;
}

int TLEManager::checksumTLELine(const char* line) {
    int sum = 0;
    
    // Sum digits and minus signs in the first TLE_CHECKSUM_DATA_LEN characters
    for (size_t i = 0; i < TLE_CHECKSUM_DATA_LEN; i++) {
        char c = line[i];
        if (c >= '0' && c <= '9') {
            sum += c - '0';
        } else if (c == '-') {
            sum += 1;
        }
    }
    
    return sum % 10;
}
