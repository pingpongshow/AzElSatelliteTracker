/**
 * @file nvs_storage.h
 * @brief Non-volatile storage management for persistent configuration
 * 
 * Handles saving and loading of calibration offsets, WiFi credentials,
 * position state, and user preferences to ESP32 NVS flash storage.
 */

#ifndef NVS_STORAGE_H
#define NVS_STORAGE_H

#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

// Stored configuration structure
struct TrackerConfig {
    // Calibration
    float azimuthOffset;
    float elevationOffset;
    
    // Last known position
    float lastAzimuth;
    float lastElevation;
    bool positionValid;
    
    // Observer location (manual override)
    double latitude;
    double longitude;
    double altitude;
    bool useManualLocation;
    
    // Tracking
    char lastSatellite[25];
    
    // Motor settings
    uint16_t motorRunCurrent;
    uint16_t motorHoldCurrent;
    uint8_t microsteps;
    uint8_t stallThreshold;
    
    // Limits
    float azMin;
    float azMax;
    float elMin;
    float elMax;
    
    // Park position
    float parkAz;
    float parkEl;
};

/**
 * @class NVSStorage
 * @brief Manages persistent storage in ESP32 NVS
 */
class NVSStorage {
public:
    NVSStorage();
    
    /**
     * @brief Initialize NVS storage
     * @return true if successful
     */
    bool begin();
    
    /**
     * @brief Load all configuration
     * @param config Output configuration structure
     * @return true if config was loaded (false = using defaults)
     */
    bool loadConfig(TrackerConfig& config);
    
    /**
     * @brief Save all configuration
     * @param config Configuration to save
     * @return true if successful
     */
    bool saveConfig(const TrackerConfig& config);
    
    /**
     * @brief Save calibration offsets
     */
    bool saveCalibration(float azOffset, float elOffset);
    
    /**
     * @brief Load calibration offsets
     */
    bool loadCalibration(float& azOffset, float& elOffset);
    
    /**
     * @brief Save current position for power-loss recovery
     */
    bool savePosition(float azimuth, float elevation);
    
    /**
     * @brief Load last saved position
     */
    bool loadPosition(float& azimuth, float& elevation);
    
    /**
     * @brief Save WiFi credentials
     */
    bool saveWiFiCredentials(const char* ssid, const char* password);
    
    /**
     * @brief Load WiFi credentials
     */
    bool loadWiFiCredentials(String& ssid, String& password);
    
    /**
     * @brief Clear WiFi credentials
     */
    void clearWiFiCredentials();
    
    /**
     * @brief Save observer location
     */
    bool saveLocation(double lat, double lon, double alt, bool useManual);
    
    /**
     * @brief Load observer location
     */
    bool loadLocation(double& lat, double& lon, double& alt, bool& useManual);
    
    /**
     * @brief Save last tracked satellite name
     */
    bool saveLastSatellite(const char* name);
    
    /**
     * @brief Load last tracked satellite name
     */
    String loadLastSatellite();
    
    /**
     * @brief Save motor current settings
     */
    bool saveMotorSettings(uint16_t runCurrent, uint16_t holdCurrent);
    
    /**
     * @brief Load motor current settings
     */
    bool loadMotorSettings(uint16_t& runCurrent, uint16_t& holdCurrent);
    
    /**
     * @brief Reset all settings to defaults
     */
    void factoryReset();
    
    /**
     * @brief Get default configuration
     */
    static TrackerConfig getDefaults();
    
    /**
     * @brief Check if first boot (no saved config)
     */
    bool isFirstBoot();

private:
    Preferences _prefs;
    bool _initialized;
    
    void setDefaults(TrackerConfig& config);
};

// Global instance
extern NVSStorage nvsStorage;

#endif // NVS_STORAGE_H
