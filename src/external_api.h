/**
 * @file external_api.h
 * @brief External API integrations (N2YO, SatNOGS, Celestrak)
 * 
 * Provides integration with external satellite tracking services:
 * - N2YO API for TLEs and visual passes
 * - SatNOGS for satellite database
 * - Celestrak for TLE data
 * 
 * ERROR HANDLING:
 * Functions use two error reporting patterns:
 * 1. For bool returns: returns false and sets _lastError (call getLastError())
 * 2. For int returns: returns 0 and sets _lastError
 * 
 * All functions use the SET_ERROR_RETURN macros from utils.h for consistency.
 */

#ifndef EXTERNAL_API_H
#define EXTERNAL_API_H

#include <Arduino.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "config.h"
#include "tle_manager.h"
#include "tracking_engine.h"
#include "utils.h"

// N2YO visual pass prediction
struct VisualPass {
    uint32_t startTime;          // Unix time
    uint32_t maxTime;            // Time of maximum elevation
    uint32_t endTime;            // Unix time
    float startAz;               // Degrees
    float startEl;               // Degrees
    float maxAz;                 // Degrees
    float maxEl;                 // Maximum elevation
    float endAz;                 // Degrees
    float endEl;                 // Degrees
    float magnitude;             // Visual magnitude (-1 = bright)
    int duration;                // Seconds
};

// SatNOGS satellite info
struct SatNOGSSatellite {
    uint32_t noradId;
    char name[50];
    char altName[50];
    char status[20];             // alive, dead, decayed, etc.
    bool hasTransmitter;
    float downlinkMHz;
    float uplinkMHz;
    char mode[20];               // FM, USB, CW, etc.
};

// SatNOGS transmitter info
struct SatNOGSTransmitter {
    uint32_t noradId;
    char description[100];
    float downlinkLow;
    float downlinkHigh;
    float uplinkLow;
    float uplinkHigh;
    char mode[20];
    bool alive;
    int baud;
};

/**
 * @class ExternalAPI
 * @brief External API client for N2YO and SatNOGS
 */
class ExternalAPI {
public:
    ExternalAPI();
    
    /**
     * @brief Set N2YO API key
     * @param apiKey Your N2YO API key
     */
    void setN2YOApiKey(const char* apiKey);
    
    /**
     * @brief Check if N2YO API key is set
     */
    bool hasN2YOApiKey() const { return _n2yoApiKey.length() > 0; }
    
    // N2YO API Methods
    
    /**
     * @brief Fetch TLE from N2YO
     * @param noradId NORAD catalog ID
     * @param entry Output TLE entry
     * @return true if successful
     */
    bool fetchTLEFromN2YO(uint32_t noradId, TLEEntry& entry);
    
    /**
     * @brief Get visual passes from N2YO
     * @param noradId NORAD catalog ID
     * @param lat Observer latitude
     * @param lon Observer longitude
     * @param alt Observer altitude (meters)
     * @param days Days to predict (max 10)
     * @param minVisibility Minimum seconds visible
     * @param passes Output array
     * @param maxPasses Maximum passes to return
     * @return Number of passes found
     */
    int getVisualPasses(uint32_t noradId, float lat, float lon, float alt,
                        int days, int minVisibility,
                        VisualPass* passes, int maxPasses);
    
    /**
     * @brief Get radio passes from N2YO
     * Similar to visual passes but for any elevation
     */
    int getRadioPasses(uint32_t noradId, float lat, float lon, float alt,
                       int days, float minEl,
                       PassInfo* passes, int maxPasses);
    
    /**
     * @brief Search satellites by name on N2YO
     * @param name Search query
     * @param results Output array of NORAD IDs
     * @param maxResults Maximum results
     * @return Number of results
     */
    int searchSatellites(const char* name, uint32_t* results, int maxResults);
    
    // SatNOGS API Methods
    
    /**
     * @brief Get satellite info from SatNOGS
     * @param noradId NORAD catalog ID
     * @param info Output satellite info
     * @return true if found
     */
    bool getSatelliteInfo(uint32_t noradId, SatNOGSSatellite& info);
    
    /**
     * @brief Get transmitter info from SatNOGS
     * @param noradId NORAD catalog ID
     * @param transmitters Output array
     * @param maxTx Maximum transmitters to return
     * @return Number of transmitters found
     */
    int getTransmitters(uint32_t noradId, SatNOGSTransmitter* transmitters, int maxTx);
    
    /**
     * @brief Search satellites on SatNOGS
     * @param query Search query
     * @param results Output array
     * @param maxResults Maximum results
     * @return Number of results
     */
    int searchSatNOGS(const char* query, SatNOGSSatellite* results, int maxResults);
    
    // Celestrak Methods
    
    /**
     * @brief Fetch TLE from Celestrak by NORAD ID
     * @param noradId NORAD catalog ID
     * @param entry Output TLE entry
     * @return true if successful
     */
    bool fetchTLEFromCelestrak(uint32_t noradId, TLEEntry& entry);
    
    /**
     * @brief Fetch TLE from Celestrak by name
     * @param name Satellite name
     * @param entry Output TLE entry
     * @return true if successful
     */
    bool fetchTLEByName(const char* name, TLEEntry& entry);
    
    /**
     * @brief Fetch TLE group from Celestrak
     * @param group Group name (amateur, stations, weather, etc.)
     * @param callback Called for each TLE
     * @return Number of TLEs fetched
     */
    int fetchTLEGroup(const char* group, void (*callback)(const TLEEntry&));
    
    /**
     * @brief Get available TLE groups
     * @param groups Output array of group names
     * @param maxGroups Maximum groups
     * @return Number of groups
     */
    int getTLEGroups(String* groups, int maxGroups);
    
    /**
     * @brief Get last error message
     */
    String getLastError() const { return _lastError; }

private:
    String _n2yoApiKey;
    String _lastError;
    HTTPClient _http;
    
    // Connection reuse tracking
    String _lastHost;           // Track last host for connection reuse
    uint32_t _lastRequestTime;  // Track last request time
    
    // Helper methods
    bool httpGet(const char* url, String& response);
    String extractHost(const char* url);
    // urlEncode moved to utils.h as shared utility
};

// Global instance
extern ExternalAPI externalAPI;

#endif // EXTERNAL_API_H
