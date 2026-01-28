/**
 * @file tracking_engine.h
 * @brief Satellite tracking engine with SGP4 orbit propagation
 * 
 * Handles TLE parsing, orbit calculation, and pass prediction
 * for satellite tracking.
 */

#ifndef TRACKING_ENGINE_H
#define TRACKING_ENGINE_H

#include <Arduino.h>
#include <Sgp4.h>
#include "config.h"
#include "gps.h"

// Tracking mode enumeration
// Note: Can't use "EXTERNAL" as Arduino.h defines it as a macro
enum class TrackingMode {
    IDLE,       // Not tracking
    MANUAL,     // Manual position control
    AUTO,       // Automatic satellite tracking
    ROTCTLD,    // External control via rotctld (renamed from EXTERNAL to avoid Arduino.h macro conflict)
    EME,        // Moon bounce (EME) tracking
    GEO         // Geostationary satellite tracking
};

// Satellite position data
struct SatellitePosition {
    float azimuth;          // Degrees from North
    float elevation;        // Degrees above horizon
    float range;            // Distance in km
    float rangeRate;        // Velocity in km/s (+ = receding)
    float latitude;         // Sub-satellite point latitude
    float longitude;        // Sub-satellite point longitude
    float altitude;         // Satellite altitude in km
    bool visible;           // Above horizon
    bool illuminated;       // In sunlight
};

// Pass information
struct PassInfo {
    uint32_t aosTime;       // Acquisition of signal (Unix time)
    uint32_t losTime;       // Loss of signal (Unix time)
    uint32_t maxElTime;     // Maximum elevation time
    float aosAz;            // AOS azimuth
    float losAz;            // LOS azimuth
    float maxEl;            // Maximum elevation
    float maxElAz;          // Azimuth at max elevation
};

// Scheduled pass for auto-tracking
struct ScheduledPass {
    PassInfo pass;
    char satellite[25];     // Satellite name
    bool autoTrack;         // Auto-start tracking at AOS
    bool prePosition;       // Pre-position before AOS
    int8_t priority;        // Priority (higher = more important)
    bool active;            // Is this slot active
};

// TLE data structure
struct TLEData {
    char name[25];          // Satellite name
    char line1[70];         // TLE line 1
    char line2[70];         // TLE line 2
    uint32_t epoch;         // TLE epoch (Unix time)
    bool valid;             // TLE validity flag
};

// Doppler shift information
struct DopplerInfo {
    float rxShiftHz;        // Receive frequency shift (Hz)
    float txShiftHz;        // Transmit frequency shift (Hz)
    float rangeRateKmS;     // Range rate in km/s
    float correctedRxMHz;   // Doppler-corrected receive freq
    float correctedTxMHz;   // Doppler-corrected transmit freq
};

// Antenna pattern for beam offset compensation
struct AntennaPattern {
    float azBeamOffset;     // Azimuth beam offset from boresight
    float elBeamOffset;     // Elevation beam offset from boresight
    float beamwidth;        // 3dB beamwidth in degrees
    bool enabled;           // Apply compensation
};

// Geostationary satellite config
struct GeoSatConfig {
    float longitude;        // Geostationary longitude
    uint32_t noradId;       // NORAD ID (optional)
    char name[25];          // Satellite name
    bool active;            // Currently tracking
};

/**
 * @class TrackingEngine
 * @brief Main satellite tracking engine
 */
class TrackingEngine {
public:
    TrackingEngine();
    
    /**
     * @brief Initialize tracking engine
     * @return true if successful
     */
    bool begin();
    
    /**
     * @brief Update tracking calculations (call frequently)
     */
    void update();
    
    /**
     * @brief Load TLE data for a satellite
     * @param name Satellite name
     * @param line1 TLE line 1
     * @param line2 TLE line 2
     * @return true if TLE is valid
     */
    bool loadTLE(const char* name, const char* line1, const char* line2);
    
    /**
     * @brief Get current satellite position
     * @return Satellite position structure
     */
    SatellitePosition getPosition() const { return _satPos; }
    
    /**
     * @brief Get next pass information
     * @param pass Output pass information
     * @return true if pass found within search window
     */
    bool getNextPass(PassInfo& pass);
    
    /**
     * @brief Calculate satellite position at a specific time
     * @param unixTime Unix timestamp
     * @param pos Output position
     * @return true if calculation successful
     */
    bool calculatePosition(uint32_t unixTime, SatellitePosition& pos);
    
    /**
     * @brief Start automatic tracking
     * @return true if tracking started
     */
    bool startTracking();
    
    /**
     * @brief Stop tracking
     */
    void stopTracking();
    
    /**
     * @brief Get current tracking mode
     */
    TrackingMode getMode() const { return _mode; }
    
    /**
     * @brief Set tracking mode
     */
    void setMode(TrackingMode mode) { _mode = mode; }
    
    /**
     * @brief Check if satellite is currently visible
     */
    bool isSatelliteVisible() const { return _satPos.visible; }
    
    /**
     * @brief Get loaded satellite name
     */
    String getSatelliteName() const { return String(_tle.name); }
    
    /**
     * @brief Check if TLE is loaded
     */
    bool hasTLE() const { return _tle.valid; }
    
    /**
     * @brief Get observer location
     */
    void getObserverLocation(double& lat, double& lon, double& alt) const;
    
    /**
     * @brief Set observer location manually
     */
    void setObserverLocation(double lat, double lon, double alt);
    
    /**
     * @brief Use GPS for observer location
     */
    void useGPSLocation() { _useGPSLocation = true; }
    
    /**
     * @brief Calculate sun position for camera alignment
     * @param az Output azimuth
     * @param el Output elevation
     * @return true if sun is above horizon
     */
    bool getSunPosition(float& az, float& el) const;
    
    /**
     * @brief Calculate moon position for camera alignment
     * @param az Output azimuth
     * @param el Output elevation
     * @return true if moon is above horizon
     */
    bool getMoonPosition(float& az, float& el) const;
    
    /**
     * @brief Check if currently tracking through zenith
     * @return true if zenith handling is active
     */
    bool isZenithPass() const { return _zenithFlipActive; }
    
    /**
     * @brief Get target azimuth (with zenith flip handling)
     */
    float getTargetAzimuth() const;
    
    /**
     * @brief Get target elevation (with zenith flip handling)
     */
    float getTargetElevation() const;
    
    // =========================================================================
    // Multi-Pass Prediction
    // =========================================================================
    
    /**
     * @brief Get multiple upcoming passes
     * @param passes Output array
     * @param maxPasses Maximum passes to return
     * @param hoursAhead Hours to search ahead
     * @param minElevation Minimum maximum elevation for pass
     * @return Number of passes found
     */
    int getPasses(PassInfo* passes, int maxPasses, int hoursAhead = PASS_SEARCH_HOURS,
                  float minElevation = PASS_MIN_ELEVATION);
    
    /**
     * @brief Schedule a pass for auto-tracking
     * @param satellite Satellite name
     * @param pass Pass information
     * @param prePosition Pre-position before AOS
     * @return true if scheduled successfully
     */
    bool schedulePass(const char* satellite, const PassInfo& pass, bool prePosition = true);
    
    /**
     * @brief Cancel a scheduled pass
     * @param index Schedule slot index
     */
    void cancelScheduledPass(int index);
    
    /**
     * @brief Get scheduled passes
     * @param passes Output array
     * @param maxPasses Maximum to return
     * @return Number of scheduled passes
     */
    int getScheduledPasses(ScheduledPass* passes, int maxPasses);
    
    /**
     * @brief Clear all scheduled passes
     */
    void clearSchedule();
    
    // =========================================================================
    // Doppler Shift Calculation
    // =========================================================================
    
    /**
     * @brief Calculate Doppler shift for current satellite
     * @param uplinkMHz Uplink frequency in MHz (0 to skip)
     * @param downlinkMHz Downlink frequency in MHz (0 to skip)
     * @return Doppler information
     */
    DopplerInfo getDoppler(float uplinkMHz, float downlinkMHz);
    
    /**
     * @brief Get current range rate
     * @return Range rate in km/s (positive = receding)
     */
    float getRangeRate() const { return _satPos.rangeRate; }
    
    // =========================================================================
    // Pre-Positioning
    // =========================================================================
    
    /**
     * @brief Enable/disable pre-positioning before passes
     * @param enable Enable pre-positioning
     * @param secondsBefore Seconds before AOS to move
     */
    void setPrePositioning(bool enable, uint32_t secondsBefore = PRE_POSITION_SECONDS);
    
    /**
     * @brief Check if currently pre-positioning
     */
    bool isPrePositioning() const { return _prePositioning; }
    
    // =========================================================================
    // Antenna Pattern Compensation
    // =========================================================================
    
    /**
     * @brief Set antenna pattern for beam offset compensation
     * @param pattern Antenna pattern parameters
     */
    void setAntennaPattern(const AntennaPattern& pattern);
    
    /**
     * @brief Get current antenna pattern
     */
    AntennaPattern getAntennaPattern() const { return _antennaPattern; }
    
    /**
     * @brief Enable/disable antenna pattern compensation
     */
    void setAntennaCompensation(bool enable) { _antennaPattern.enabled = enable; }
    
    // =========================================================================
    // EME (Moon Bounce) Mode
    // =========================================================================
    
    /**
     * @brief Enable EME tracking mode
     * @return true if moon is above horizon
     */
    bool enableEMEMode();
    
    /**
     * @brief Disable EME mode
     */
    void disableEMEMode();
    
    /**
     * @brief Check if in EME mode
     */
    bool isEMEMode() const { return _mode == TrackingMode::EME; }
    
    /**
     * @brief Get moon distance in km
     */
    float getMoonDistance() const { return _moonDistance; }
    
    /**
     * @brief Calculate EME path loss
     * @param freqMHz Frequency in MHz
     * @return Path loss in dB
     */
    float getEMEPathLoss(float freqMHz) const;
    
    // =========================================================================
    // Geostationary Satellite Mode
    // =========================================================================
    
    /**
     * @brief Point at geostationary position
     * @param longitude Geostationary longitude
     * @param name Optional satellite name
     * @return true if position is visible
     */
    bool pointToGeo(float longitude, const char* name = nullptr);
    
    /**
     * @brief Enable geo tracking with station keeping corrections
     * @param config Geo satellite configuration
     * @return true if successful
     */
    bool enableGeoTracking(const GeoSatConfig& config);
    
    /**
     * @brief Disable geo tracking
     */
    void disableGeoTracking();
    
    /**
     * @brief Check if in geo mode
     */
    bool isGeoMode() const { return _mode == TrackingMode::GEO; }
    
    /**
     * @brief Calculate azimuth/elevation for geostationary position
     * @param geoLongitude Geostationary longitude
     * @param az Output azimuth
     * @param el Output elevation
     * @return true if position is visible
     */
    bool calculateGeoPosition(float geoLongitude, float& az, float& el);

private:
    Sgp4 _sgp4;
    TLEData _tle;
    SatellitePosition _satPos;
    TrackingMode _mode;
    
    // Observer location
    bool _useGPSLocation;
    double _observerLat;
    double _observerLon;
    double _observerAlt;
    
    // Cached SGP4 site location to avoid redundant updates
    double _cachedSiteLat;
    double _cachedSiteLon;
    double _cachedSiteAlt;
    bool _siteInitialized;
    
    // Zenith flip handling
    bool _zenithFlipActive;
    bool _zenithFlipDirection;  // true = flipped
    float _lastAzimuth;
    
    // Timing
    uint32_t _lastUpdateMs;
    uint32_t _lastCalculationMs;
    
    // GPS time tracking for fallback
    uint32_t _lastValidGpsTime;      // Last known valid Unix time from GPS
    uint32_t _lastValidGpsMillis;    // millis() when _lastValidGpsTime was captured
    bool _timeIsEstimated;           // True if using estimated time
    
    // Range rate calculation (for Doppler)
    float _lastRange;                // Previous range for rate calculation
    uint32_t _lastRangeTime;         // Unix time of previous range measurement
    
    // Multi-pass scheduling
    ScheduledPass _schedule[MAX_SCHEDULED_PASSES];
    int _scheduleCount;
    
    // Pre-positioning
    bool _prePositionEnabled;
    uint32_t _prePositionSeconds;
    bool _prePositioning;
    
    // Antenna pattern
    AntennaPattern _antennaPattern;
    
    // EME mode
    float _moonDistance;
    
    // Geo mode
    GeoSatConfig _geoConfig;
    uint32_t _lastGeoUpdate;
    
    // Internal methods
    void updateObserverLocation();
    void handleZenithFlip();
    void checkScheduledPasses();
    void handlePrePositioning();
    double julianDateFromUnix(uint32_t unixTime) const;
    void calculateSunPosition(double jd, double lat, double lon, float& az, float& el) const;
    void calculateMoonPosition(double jd, double lat, double lon, float& az, float& el) const;
    void calculateMoonPositionHighPrecision(double jd, double lat, double lon, 
                                            float& az, float& el, float& distance) const;
    float applyAntennaPattern(float az, float el, bool isAzimuth) const;
};

// Global instance
extern TrackingEngine trackingEngine;

#endif // TRACKING_ENGINE_H
