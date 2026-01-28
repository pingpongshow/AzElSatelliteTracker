/**
 * @file tle_manager.h
 * @brief TLE (Two-Line Element) fetching and storage management
 * 
 * Handles downloading TLEs from Celestrak and storing them in
 * non-volatile storage for satellite tracking.
 * 
 * ERROR HANDLING:
 * Functions use two error reporting patterns:
 * 1. For bool returns: returns false and sets _lastError (call getLastError())
 * 2. For int returns: returns 0 or -1 and sets _lastError
 * 
 * All functions use the SET_ERROR_RETURN macros from utils.h for consistency.
 */

#ifndef TLE_MANAGER_H
#define TLE_MANAGER_H

#include <Arduino.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include "config.h"
#include "utils.h"

// TLE entry structure for storage
struct TLEEntry {
    char name[25];
    char line1[70];
    char line2[70];
    uint32_t fetchTime;     // When this TLE was fetched
    bool valid;
};

/**
 * @class TLEManager
 * @brief Manages TLE data fetching and storage
 */
class TLEManager {
public:
    TLEManager();
    
    /**
     * @brief Initialize TLE manager
     * @return true if successful
     */
    bool begin();
    
    /**
     * @brief Fetch TLE from Celestrak by NORAD ID
     * @param noradId NORAD catalog number
     * @param entry Output TLE entry
     * @return true if fetch successful
     */
    bool fetchByNoradId(uint32_t noradId, TLEEntry& entry);
    
    /**
     * @brief Fetch TLE from Celestrak by satellite name
     * @param name Satellite name (case insensitive)
     * @param entry Output TLE entry
     * @return true if fetch successful
     */
    bool fetchByName(const char* name, TLEEntry& entry);
    
    /**
     * @brief Fetch amateur satellite TLEs
     * @param callback Function called for each TLE found
     * @return Number of TLEs fetched
     */
    int fetchAmateurSatellites(void (*callback)(const TLEEntry& entry));
    
    /**
     * @brief Save TLE to non-volatile storage
     * @param entry TLE to save
     * @param slot Storage slot (0 to MAX_STORED_TLES-1)
     * @return true if saved successfully
     */
    bool saveTLE(const TLEEntry& entry, int slot);
    
    /**
     * @brief Load TLE from non-volatile storage
     * @param slot Storage slot
     * @param entry Output TLE entry
     * @return true if loaded successfully
     */
    bool loadTLE(int slot, TLEEntry& entry);
    
    /**
     * @brief Delete TLE from storage by slot
     * @param slot Storage slot
     */
    void deleteTLE(int slot);
    
    /**
     * @brief Delete TLE from storage by name
     * @param name Satellite name (case insensitive)
     * @return true if found and deleted, false if not found
     */
    bool deleteTLE(const char* name);
    
    /**
     * @brief Get number of stored TLEs
     */
    int getStoredCount();
    
    /**
     * @brief List all stored TLE names
     * @param names Array to fill with names
     * @param maxCount Maximum entries to return
     * @return Number of entries returned
     */
    int listStoredTLEs(String* names, int maxCount);
    
    /**
     * @brief Find stored TLE by name
     * @param name Satellite name
     * @param entry Output TLE entry
     * @return true if found
     */
    bool findStoredTLE(const char* name, TLEEntry& entry);
    
    /**
     * @brief Parse TLE from raw text
     * @param text Raw TLE text (3 lines)
     * @param entry Output TLE entry
     * @return true if parsing successful
     */
    bool parseTLE(const char* text, TLEEntry& entry);
    
    /**
     * @brief Check if TLE is stale (older than 7 days)
     * @param entry TLE to check
     * @return true if stale
     */
    bool isTLEStale(const TLEEntry& entry);
    
    /**
     * @brief Find first empty storage slot (O(1) using bitmap)
     * @return Slot index or -1 if all full
     */
    int findEmptySlot();
    
    /**
     * @brief Find slot containing TLE with given name
     * @param name Satellite name to find
     * @return Slot index or -1 if not found
     */
    int findSlotByName(const char* name);
    
    /**
     * @brief Get last fetch error message
     */
    String getLastError() const { return _lastError; }
    
    /**
     * @brief Check if network is available for fetching
     */
    bool isNetworkAvailable();

private:
    Preferences _prefs;
    String _lastError;
    
    // HTTP client member for connection reuse
    HTTPClient _http;
    String _lastHost;           // Track last host for connection reuse
    uint32_t _lastRequestTime;  // Track last request time
    
    // Slot occupancy bitmap for O(1) empty slot finding
    // Each bit represents a slot: 1 = occupied, 0 = empty
    uint32_t _slotBitmap;
    bool _slotBitmapInitialized;
    
    // HTTP fetch helper
    bool httpFetch(const char* url, String& response);
    
    // Extract host from URL for connection reuse tracking
    String extractHost(const char* url);
    
    // Slot bitmap helpers (internal)
    void initSlotBitmap();
    void markSlotOccupied(int slot);
    void markSlotEmpty(int slot);
    
    // TLE validation
    bool validateTLE(const char* line1, const char* line2);
    int checksumTLELine(const char* line);
};

// Global instance
extern TLEManager tleManager;

#endif // TLE_MANAGER_H
