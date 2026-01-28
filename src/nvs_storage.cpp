/**
 * @file nvs_storage.cpp
 * @brief Non-volatile storage implementation
 */

#include "nvs_storage.h"

// Global instance
NVSStorage nvsStorage;

// NVS Keys
static const char* KEY_INITIALIZED = "init";
static const char* KEY_AZ_OFFSET = "az_off";
static const char* KEY_EL_OFFSET = "el_off";
static const char* KEY_LAST_AZ = "last_az";
static const char* KEY_LAST_EL = "last_el";
static const char* KEY_POS_VALID = "pos_valid";
static const char* KEY_LATITUDE = "lat";
static const char* KEY_LONGITUDE = "lon";
static const char* KEY_ALTITUDE = "alt";
static const char* KEY_MANUAL_LOC = "manual_loc";
static const char* KEY_LAST_SAT = "last_sat";
static const char* KEY_RUN_CURRENT = "run_curr";
static const char* KEY_HOLD_CURRENT = "hold_curr";
static const char* KEY_MICROSTEPS = "usteps";
static const char* KEY_STALL_THRESH = "stall_th";
static const char* KEY_AZ_MIN = "az_min";
static const char* KEY_AZ_MAX = "az_max";
static const char* KEY_EL_MIN = "el_min";
static const char* KEY_EL_MAX = "el_max";
static const char* KEY_PARK_AZ = "park_az";
static const char* KEY_PARK_EL = "park_el";
static const char* KEY_WIFI_SSID = "wifi_ssid";
static const char* KEY_WIFI_PASS = "wifi_pass";

NVSStorage::NVSStorage()
    : _initialized(false)
{
}

bool NVSStorage::begin() {
    DEBUG_PRINTLN("NVSStorage: Initializing...");
    
    if (!_prefs.begin(NVS_NAMESPACE, false)) {
        DEBUG_PRINTLN("NVSStorage: Failed to open preferences");
        return false;
    }
    
    _initialized = true;
    
    if (isFirstBoot()) {
        DEBUG_PRINTLN("NVSStorage: First boot detected, saving defaults");
        TrackerConfig defaults = getDefaults();
        saveConfig(defaults);
        _prefs.putBool(KEY_INITIALIZED, true);
    }
    
    DEBUG_PRINTLN("NVSStorage: Initialization complete");
    return true;
}

bool NVSStorage::loadConfig(TrackerConfig& config) {
    if (!_initialized) return false;
    
    // Load calibration
    config.azimuthOffset = _prefs.getFloat(KEY_AZ_OFFSET, 0.0f);
    config.elevationOffset = _prefs.getFloat(KEY_EL_OFFSET, 0.0f);
    
    // Load position
    config.lastAzimuth = _prefs.getFloat(KEY_LAST_AZ, PARK_AZ_DEG);
    config.lastElevation = _prefs.getFloat(KEY_LAST_EL, PARK_EL_DEG);
    config.positionValid = _prefs.getBool(KEY_POS_VALID, false);
    
    // Load location
    config.latitude = _prefs.getDouble(KEY_LATITUDE, DEFAULT_LATITUDE);
    config.longitude = _prefs.getDouble(KEY_LONGITUDE, DEFAULT_LONGITUDE);
    config.altitude = _prefs.getDouble(KEY_ALTITUDE, DEFAULT_ALTITUDE);
    config.useManualLocation = _prefs.getBool(KEY_MANUAL_LOC, false);
    
    // Load satellite
    String sat = _prefs.getString(KEY_LAST_SAT, "");
    strncpy(config.lastSatellite, sat.c_str(), sizeof(config.lastSatellite) - 1);
    config.lastSatellite[sizeof(config.lastSatellite) - 1] = '\0';  // Ensure null termination
    
    // Load motor settings
    config.motorRunCurrent = _prefs.getUShort(KEY_RUN_CURRENT, MOTOR_RUN_CURRENT_MA);
    config.motorHoldCurrent = _prefs.getUShort(KEY_HOLD_CURRENT, MOTOR_HOLD_CURRENT_MA);
    config.microsteps = _prefs.getUChar(KEY_MICROSTEPS, TMC_MICROSTEPS);
    config.stallThreshold = _prefs.getUChar(KEY_STALL_THRESH, TMC_STALL_THRESHOLD);
    
    // Load limits
    config.azMin = _prefs.getFloat(KEY_AZ_MIN, AZ_MIN_DEG);
    config.azMax = _prefs.getFloat(KEY_AZ_MAX, AZ_MAX_DEG);
    config.elMin = _prefs.getFloat(KEY_EL_MIN, EL_MIN_DEG);
    config.elMax = _prefs.getFloat(KEY_EL_MAX, EL_MAX_DEG);
    
    // Load park position
    config.parkAz = _prefs.getFloat(KEY_PARK_AZ, PARK_AZ_DEG);
    config.parkEl = _prefs.getFloat(KEY_PARK_EL, PARK_EL_DEG);
    
    DEBUG_PRINTLN("NVSStorage: Configuration loaded");
    return true;
}

bool NVSStorage::saveConfig(const TrackerConfig& config) {
    if (!_initialized) return false;
    
    // Save calibration
    _prefs.putFloat(KEY_AZ_OFFSET, config.azimuthOffset);
    _prefs.putFloat(KEY_EL_OFFSET, config.elevationOffset);
    
    // Save position
    _prefs.putFloat(KEY_LAST_AZ, config.lastAzimuth);
    _prefs.putFloat(KEY_LAST_EL, config.lastElevation);
    _prefs.putBool(KEY_POS_VALID, config.positionValid);
    
    // Save location
    _prefs.putDouble(KEY_LATITUDE, config.latitude);
    _prefs.putDouble(KEY_LONGITUDE, config.longitude);
    _prefs.putDouble(KEY_ALTITUDE, config.altitude);
    _prefs.putBool(KEY_MANUAL_LOC, config.useManualLocation);
    
    // Save satellite
    _prefs.putString(KEY_LAST_SAT, config.lastSatellite);
    
    // Save motor settings
    _prefs.putUShort(KEY_RUN_CURRENT, config.motorRunCurrent);
    _prefs.putUShort(KEY_HOLD_CURRENT, config.motorHoldCurrent);
    _prefs.putUChar(KEY_MICROSTEPS, config.microsteps);
    _prefs.putUChar(KEY_STALL_THRESH, config.stallThreshold);
    
    // Save limits
    _prefs.putFloat(KEY_AZ_MIN, config.azMin);
    _prefs.putFloat(KEY_AZ_MAX, config.azMax);
    _prefs.putFloat(KEY_EL_MIN, config.elMin);
    _prefs.putFloat(KEY_EL_MAX, config.elMax);
    
    // Save park position
    _prefs.putFloat(KEY_PARK_AZ, config.parkAz);
    _prefs.putFloat(KEY_PARK_EL, config.parkEl);
    
    DEBUG_PRINTLN("NVSStorage: Configuration saved");
    return true;
}

bool NVSStorage::saveCalibration(float azOffset, float elOffset) {
    if (!_initialized) return false;
    
    _prefs.putFloat(KEY_AZ_OFFSET, azOffset);
    _prefs.putFloat(KEY_EL_OFFSET, elOffset);
    
    DEBUG_PRINTF("NVSStorage: Calibration saved: Az=%.3f, El=%.3f\n", azOffset, elOffset);
    return true;
}

bool NVSStorage::loadCalibration(float& azOffset, float& elOffset) {
    if (!_initialized) return false;
    
    azOffset = _prefs.getFloat(KEY_AZ_OFFSET, 0.0f);
    elOffset = _prefs.getFloat(KEY_EL_OFFSET, 0.0f);
    
    return true;
}

bool NVSStorage::savePosition(float azimuth, float elevation) {
    if (!_initialized) return false;
    
    _prefs.putFloat(KEY_LAST_AZ, azimuth);
    _prefs.putFloat(KEY_LAST_EL, elevation);
    _prefs.putBool(KEY_POS_VALID, true);
    
    return true;
}

bool NVSStorage::loadPosition(float& azimuth, float& elevation) {
    if (!_initialized) return false;
    
    if (!_prefs.getBool(KEY_POS_VALID, false)) {
        return false;
    }
    
    azimuth = _prefs.getFloat(KEY_LAST_AZ, PARK_AZ_DEG);
    elevation = _prefs.getFloat(KEY_LAST_EL, PARK_EL_DEG);
    
    return true;
}

bool NVSStorage::saveWiFiCredentials(const char* ssid, const char* password) {
    if (!_initialized) return false;
    
    _prefs.putString(KEY_WIFI_SSID, ssid);
    _prefs.putString(KEY_WIFI_PASS, password);
    
    DEBUG_PRINTF("NVSStorage: WiFi credentials saved for '%s'\n", ssid);
    return true;
}

bool NVSStorage::loadWiFiCredentials(String& ssid, String& password) {
    if (!_initialized) return false;
    
    ssid = _prefs.getString(KEY_WIFI_SSID, "");
    password = _prefs.getString(KEY_WIFI_PASS, "");
    
    return ssid.length() > 0;
}

void NVSStorage::clearWiFiCredentials() {
    if (!_initialized) return;
    
    _prefs.remove(KEY_WIFI_SSID);
    _prefs.remove(KEY_WIFI_PASS);
    
    DEBUG_PRINTLN("NVSStorage: WiFi credentials cleared");
}

bool NVSStorage::saveLocation(double lat, double lon, double alt, bool useManual) {
    if (!_initialized) return false;
    
    _prefs.putDouble(KEY_LATITUDE, lat);
    _prefs.putDouble(KEY_LONGITUDE, lon);
    _prefs.putDouble(KEY_ALTITUDE, alt);
    _prefs.putBool(KEY_MANUAL_LOC, useManual);
    
    DEBUG_PRINTF("NVSStorage: Location saved: %.6f, %.6f, %.1fm (manual=%d)\n",
                 lat, lon, alt, useManual);
    return true;
}

bool NVSStorage::loadLocation(double& lat, double& lon, double& alt, bool& useManual) {
    if (!_initialized) return false;
    
    lat = _prefs.getDouble(KEY_LATITUDE, DEFAULT_LATITUDE);
    lon = _prefs.getDouble(KEY_LONGITUDE, DEFAULT_LONGITUDE);
    alt = _prefs.getDouble(KEY_ALTITUDE, DEFAULT_ALTITUDE);
    useManual = _prefs.getBool(KEY_MANUAL_LOC, false);
    
    return true;
}

bool NVSStorage::saveLastSatellite(const char* name) {
    if (!_initialized) return false;
    
    _prefs.putString(KEY_LAST_SAT, name);
    return true;
}

String NVSStorage::loadLastSatellite() {
    if (!_initialized) return "";
    
    return _prefs.getString(KEY_LAST_SAT, "");
}

bool NVSStorage::saveMotorSettings(uint16_t runCurrent, uint16_t holdCurrent) {
    if (!_initialized) return false;
    
    _prefs.putUShort(KEY_RUN_CURRENT, runCurrent);
    _prefs.putUShort(KEY_HOLD_CURRENT, holdCurrent);
    
    return true;
}

bool NVSStorage::loadMotorSettings(uint16_t& runCurrent, uint16_t& holdCurrent) {
    if (!_initialized) return false;
    
    runCurrent = _prefs.getUShort(KEY_RUN_CURRENT, MOTOR_RUN_CURRENT_MA);
    holdCurrent = _prefs.getUShort(KEY_HOLD_CURRENT, MOTOR_HOLD_CURRENT_MA);
    
    return true;
}

void NVSStorage::factoryReset() {
    if (!_initialized) return;
    
    DEBUG_PRINTLN("NVSStorage: Factory reset");
    _prefs.clear();
    
    // Save defaults
    TrackerConfig defaults = getDefaults();
    saveConfig(defaults);
    _prefs.putBool(KEY_INITIALIZED, true);
}

TrackerConfig NVSStorage::getDefaults() {
    TrackerConfig config;
    
    config.azimuthOffset = 0.0f;
    config.elevationOffset = 0.0f;
    config.lastAzimuth = PARK_AZ_DEG;
    config.lastElevation = PARK_EL_DEG;
    config.positionValid = false;
    config.latitude = DEFAULT_LATITUDE;
    config.longitude = DEFAULT_LONGITUDE;
    config.altitude = DEFAULT_ALTITUDE;
    config.useManualLocation = false;
    config.lastSatellite[0] = '\0';
    config.motorRunCurrent = MOTOR_RUN_CURRENT_MA;
    config.motorHoldCurrent = MOTOR_HOLD_CURRENT_MA;
    config.microsteps = TMC_MICROSTEPS;
    config.stallThreshold = TMC_STALL_THRESHOLD;
    config.azMin = AZ_MIN_DEG;
    config.azMax = AZ_MAX_DEG;
    config.elMin = EL_MIN_DEG;
    config.elMax = EL_MAX_DEG;
    config.parkAz = PARK_AZ_DEG;
    config.parkEl = PARK_EL_DEG;
    
    return config;
}

bool NVSStorage::isFirstBoot() {
    return !_prefs.getBool(KEY_INITIALIZED, false);
}
