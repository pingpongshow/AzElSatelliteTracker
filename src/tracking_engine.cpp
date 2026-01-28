/**
 * @file tracking_engine.cpp
 * @brief Satellite tracking engine implementation
 */

#include "tracking_engine.h"
#include "stepper_control.h"
#include <math.h>

// Global instance
TrackingEngine trackingEngine;

// Constants for astronomical calculations
// Note: DEG_TO_RAD and RAD_TO_DEG are already defined in Arduino.h
const double UNIX_TO_JD_OFFSET = 2440587.5;  // Unix epoch in Julian Date
const double SECONDS_PER_DAY = 86400.0;

TrackingEngine::TrackingEngine()
    : _mode(TrackingMode::IDLE)
    , _useGPSLocation(true)
    , _observerLat(DEFAULT_LATITUDE)
    , _observerLon(DEFAULT_LONGITUDE)
    , _observerAlt(DEFAULT_ALTITUDE)
    , _cachedSiteLat(0)
    , _cachedSiteLon(0)
    , _cachedSiteAlt(0)
    , _siteInitialized(false)
    , _zenithFlipActive(false)
    , _zenithFlipDirection(false)
    , _lastAzimuth(0)
    , _lastUpdateMs(0)
    , _lastCalculationMs(0)
    , _lastValidGpsTime(0)
    , _lastValidGpsMillis(0)
    , _timeIsEstimated(true)
    , _lastRange(0)
    , _lastRangeTime(0)
    , _scheduleCount(0)
    , _prePositionEnabled(true)
    , _prePositionSeconds(PRE_POSITION_SECONDS)
    , _prePositioning(false)
    , _moonDistance(EME_AVG_DISTANCE_KM)
    , _lastGeoUpdate(0)
{
    memset(&_tle, 0, sizeof(TLEData));
    memset(&_satPos, 0, sizeof(SatellitePosition));
    memset(&_schedule, 0, sizeof(_schedule));
    memset(&_geoConfig, 0, sizeof(GeoSatConfig));
    
    // Default antenna pattern (no compensation)
    _antennaPattern.azBeamOffset = DEFAULT_AZ_BEAM_OFFSET;
    _antennaPattern.elBeamOffset = DEFAULT_EL_BEAM_OFFSET;
    _antennaPattern.beamwidth = DEFAULT_BEAMWIDTH;
    _antennaPattern.enabled = false;
}

bool TrackingEngine::begin() {
    DEBUG_PRINTLN("TrackingEngine: Initializing...");
    
    // Set initial observer location from GPS or defaults
    updateObserverLocation();
    
    _lastUpdateMs = millis();
    
    DEBUG_PRINTLN("TrackingEngine: Initialization complete");
    return true;
}

void TrackingEngine::update() {
    uint32_t now = millis();
    
    // Update observer location from GPS if enabled
    if (_useGPSLocation && gps.hasFix()) {
        updateObserverLocation();
    }
    
    // Check scheduled passes
    checkScheduledPasses();
    
    // Handle pre-positioning
    handlePrePositioning();
    
    // Rate-limit calculations
    if (now - _lastCalculationMs < TRACKING_UPDATE_INTERVAL_MS) return;
    
    // Get current time - prefer GPS, fallback to extrapolation
    uint32_t currentTime = gps.getUnixTime();
    if (currentTime != 0) {
        // Valid GPS time - update our reference
        _lastValidGpsTime = currentTime;
        _lastValidGpsMillis = now;
        _timeIsEstimated = false;
    } else if (_lastValidGpsTime != 0) {
        // No GPS time but we have a previous reference - extrapolate
        uint32_t elapsedSeconds = (now - _lastValidGpsMillis) / 1000;
        currentTime = _lastValidGpsTime + elapsedSeconds;
        _timeIsEstimated = true;
        
        // Warn if extrapolating for too long (rate-limited to once per minute)
        static uint32_t lastExtrapolationWarn = 0;
        if (elapsedSeconds > GPS_EXTRAPOLATION_WARN_SEC && (now - lastExtrapolationWarn > 60000)) {
            DEBUG_PRINTLN("TrackingEngine: WARNING - Using estimated time (GPS unavailable for >10min)");
            lastExtrapolationWarn = now;
        }
    } else {
        // No GPS time and no reference - cannot track accurately
        // Rate-limit this message to once every 30 seconds
        static uint32_t lastNoTimeWarn = 0;
        if (now - lastNoTimeWarn > 30000) {
            DEBUG_PRINTLN("TrackingEngine: WARNING - No valid time source (waiting for GPS or NTP)");
            lastNoTimeWarn = now;
        }
        return;  // Skip tracking update - SGP4 will produce garbage
    }
    
    // Handle different tracking modes
    switch (_mode) {
        case TrackingMode::AUTO:
            // Only calculate position if we have a valid TLE
            if (_tle.valid) {
                SatellitePosition pos;
                if (calculatePosition(currentTime, pos)) {
                    _satPos = pos;
                    
                    // Handle zenith passes during tracking
                    handleZenithFlip();
                    
                    // Update stepper targets if satellite is visible
                    if (_satPos.visible) {
                        float targetAz = getTargetAzimuth();
                        float targetEl = getTargetElevation();
                        
                        // Apply antenna pattern compensation
                        // Store original values to avoid using modified az for el calculation
                        if (_antennaPattern.enabled) {
                            float origAz = targetAz;
                            float origEl = targetEl;
                            targetAz = applyAntennaPattern(origAz, origEl, true);
                            targetEl = applyAntennaPattern(origAz, origEl, false);
                        }
                        
                        stepperControl.moveTo(targetAz, targetEl, TRACKING_SPEED_DEG_S);
                    }
                }
            }
            break;
            
        case TrackingMode::EME: {
            // Track the moon
            float moonAz, moonEl;
            calculateMoonPositionHighPrecision(
                julianDateFromUnix(currentTime),
                _observerLat, _observerLon,
                moonAz, moonEl, _moonDistance
            );
            
            if (moonEl > 0) {
                // Apply antenna pattern compensation
                // Store original values to avoid using modified az for el calculation
                if (_antennaPattern.enabled) {
                    float origAz = moonAz;
                    float origEl = moonEl;
                    moonAz = applyAntennaPattern(origAz, origEl, true);
                    moonEl = applyAntennaPattern(origAz, origEl, false);
                }
                
                stepperControl.moveTo(moonAz, moonEl, EME_TRACKING_RATE);
            }
            break;
        }
            
        case TrackingMode::GEO: {
            // Check if we need to update geo position
            if (now - _lastGeoUpdate >= GEO_UPDATE_INTERVAL_MS) {
                float geoAz, geoEl;
                if (calculateGeoPosition(_geoConfig.longitude, geoAz, geoEl)) {
                    // Check if position changed significantly
                    float currentAz = stepperControl.getAzimuth();
                    float currentEl = stepperControl.getElevation();
                    
                    if (abs(geoAz - currentAz) > GEO_POSITION_TOLERANCE ||
                        abs(geoEl - currentEl) > GEO_POSITION_TOLERANCE) {
                        
                        // Apply antenna pattern compensation
                        // Store original values to avoid using modified az for el calculation
                        if (_antennaPattern.enabled) {
                            float origAz = geoAz;
                            float origEl = geoEl;
                            geoAz = applyAntennaPattern(origAz, origEl, true);
                            geoEl = applyAntennaPattern(origAz, origEl, false);
                        }
                        
                        stepperControl.moveTo(geoAz, geoEl, TRACKING_SPEED_DEG_S);
                    }
                }
                _lastGeoUpdate = now;
            }
            break;
        }
            
        default:
            // IDLE, MANUAL, ROTCTLD - no automatic updates
            break;
    }
    
    _lastCalculationMs = now;
}

bool TrackingEngine::loadTLE(const char* name, const char* line1, const char* line2) {
    DEBUG_PRINTF("TrackingEngine: Loading TLE for %s\n", name);
    
    // Validate TLE format
    if (strlen(line1) < 69 || strlen(line2) < 69) {
        DEBUG_PRINTLN("TrackingEngine: Invalid TLE line length");
        return false;
    }
    
    if (line1[0] != '1' || line2[0] != '2') {
        DEBUG_PRINTLN("TrackingEngine: Invalid TLE line numbers");
        return false;
    }
    
    // Store TLE data
    strncpy(_tle.name, name, sizeof(_tle.name) - 1);
    strncpy(_tle.line1, line1, sizeof(_tle.line1) - 1);
    strncpy(_tle.line2, line2, sizeof(_tle.line2) - 1);
    
    // Initialize SGP4 site and update cache
    double altKm = _observerAlt / 1000.0;
    _sgp4.site(_observerLat, _observerLon, altKm);
    _cachedSiteLat = _observerLat;
    _cachedSiteLon = _observerLon;
    _cachedSiteAlt = altKm;
    _siteInitialized = true;
    
    if (!_sgp4.init(_tle.name, _tle.line1, _tle.line2)) {
        DEBUG_PRINTLN("TrackingEngine: SGP4 initialization failed");
        _tle.valid = false;
        return false;
    }
    
    _tle.valid = true;
    
    // Reset zenith flip state
    _zenithFlipActive = false;
    _zenithFlipDirection = false;
    
    // Reset range rate tracking for fresh Doppler calculations
    _lastRange = 0;
    _lastRangeTime = 0;
    
    DEBUG_PRINTF("TrackingEngine: TLE loaded successfully for %s\n", name);
    return true;
}

bool TrackingEngine::calculatePosition(uint32_t unixTime, SatellitePosition& pos) {
    if (!_tle.valid) return false;
    
    // Convert Unix time to Julian Date
    double jd = julianDateFromUnix(unixTime);
    
    // Only update SGP4 site when observer location has changed significantly
    // Tolerance: ~10 meters (0.0001 degrees lat/lon, 0.01 km altitude)
    // This avoids unnecessary SGP4 recomputation on every calculation
    const double LAT_LON_TOLERANCE = 0.0001;  // ~10m at equator
    const double ALT_TOLERANCE_KM = 0.01;      // 10m in km
    
    double altKm = _observerAlt / 1000.0;
    bool locationChanged = !_siteInitialized ||
        fabs(_cachedSiteLat - _observerLat) > LAT_LON_TOLERANCE ||
        fabs(_cachedSiteLon - _observerLon) > LAT_LON_TOLERANCE ||
        fabs(_cachedSiteAlt - altKm) > ALT_TOLERANCE_KM;
    
    if (locationChanged) {
        _sgp4.site(_observerLat, _observerLon, altKm);
        _cachedSiteLat = _observerLat;
        _cachedSiteLon = _observerLon;
        _cachedSiteAlt = altKm;
        _siteInitialized = true;
    }
    
    // Calculate satellite position
    _sgp4.findsat(jd);
    
    // Get topocentric coordinates (relative to observer)
    pos.azimuth = _sgp4.satAz;
    pos.elevation = _sgp4.satEl;
    pos.range = _sgp4.satDist;
    
    // Calculate range rate from consecutive positions (for Doppler)
    // Range rate = change in range / change in time (km/s)
    if (_lastRangeTime != 0 && unixTime > _lastRangeTime) {
        float dt = (float)(unixTime - _lastRangeTime);  // seconds
        if (dt > 0 && dt < 10.0f) {  // Reasonable time delta (avoid stale data)
            pos.rangeRate = (pos.range - _lastRange) / dt;
        } else {
            pos.rangeRate = 0.0f;  // Reset on large time gaps
        }
    } else {
        pos.rangeRate = 0.0f;
    }
    
    // Store current range for next calculation
    _lastRange = pos.range;
    _lastRangeTime = unixTime;
    
    // Get satellite geodetic coordinates
    pos.latitude = _sgp4.satLat;
    pos.longitude = _sgp4.satLon;
    pos.altitude = _sgp4.satAlt;
    
    // Determine visibility
    pos.visible = (pos.elevation > 0);
    
    // Check if satellite is illuminated (simplified)
    // A more accurate calculation would check Earth's shadow
    float sunAz, sunEl;
    getSunPosition(sunAz, sunEl);
    pos.illuminated = (sunEl > -6);  // Astronomical twilight
    
    return true;
}

bool TrackingEngine::getNextPass(PassInfo& pass) {
    if (!_tle.valid) return false;
    
    uint32_t now = gps.getUnixTime();
    if (now == 0) return false;
    
    // Search for next pass (up to 24 hours ahead)
    uint32_t searchEnd = now + 86400;
    uint32_t step = 60;  // 1 minute steps
    
    bool inPass = false;
    SatellitePosition pos;
    SatellitePosition prevPos;  // Cache previous position for boundary refinement
    bool havePrevPos = false;
    
    for (uint32_t t = now; t < searchEnd; t += step) {
        if (!calculatePosition(t, pos)) continue;
        
        if (!inPass && pos.elevation > 0) {
            // Found AOS
            inPass = true;
            pass.aosTime = t;
            pass.aosAz = pos.azimuth;
            pass.maxEl = pos.elevation;
            pass.maxElTime = t;
            pass.maxElAz = pos.azimuth;
            
            // Refine AOS time using binary search (more efficient than linear)
            // We know: at t-step elevation <= 0, at t elevation > 0
            if (havePrevPos && prevPos.elevation <= 0) {
                uint32_t lo = t - step;
                uint32_t hi = t;
                SatellitePosition midPos;
                
                while (hi - lo > 1) {
                    uint32_t mid = (lo + hi) / 2;
                    if (calculatePosition(mid, midPos)) {
                        if (midPos.elevation > 0) {
                            hi = mid;
                            pass.aosTime = mid;
                            pass.aosAz = midPos.azimuth;
                        } else {
                            lo = mid;
                        }
                    } else {
                        break;  // Calculation failed, use current best
                    }
                }
            }
        } else if (inPass) {
            // Track maximum elevation
            if (pos.elevation > pass.maxEl) {
                pass.maxEl = pos.elevation;
                pass.maxElTime = t;
                pass.maxElAz = pos.azimuth;
            }
            
            if (pos.elevation <= 0) {
                // Found LOS
                pass.losTime = t;
                pass.losAz = pos.azimuth;
                
                // Refine LOS time using binary search
                // We know: at t-step elevation > 0 (prevPos), at t elevation <= 0
                if (havePrevPos && prevPos.elevation > 0) {
                    uint32_t lo = t - step;
                    uint32_t hi = t;
                    SatellitePosition midPos;
                    
                    // Find last moment with positive elevation
                    pass.losTime = lo;
                    pass.losAz = prevPos.azimuth;
                    
                    while (hi - lo > 1) {
                        uint32_t mid = (lo + hi) / 2;
                        if (calculatePosition(mid, midPos)) {
                            if (midPos.elevation > 0) {
                                lo = mid;
                                pass.losTime = mid;
                                pass.losAz = midPos.azimuth;
                            } else {
                                hi = mid;
                            }
                        } else {
                            break;  // Calculation failed, use current best
                        }
                    }
                }
                
                return true;
            }
        }
        
        // Cache current position for next iteration's boundary refinement
        prevPos = pos;
        havePrevPos = true;
    }
    
    return false;
}

bool TrackingEngine::startTracking() {
    if (!_tle.valid) {
        DEBUG_PRINTLN("TrackingEngine: Cannot start tracking - no TLE loaded");
        return false;
    }
    
    if (!gps.hasFix() && !gps.isUsingManualPosition()) {
        DEBUG_PRINTLN("TrackingEngine: Cannot start tracking - no GPS fix");
        return false;
    }
    
    _mode = TrackingMode::AUTO;
    _zenithFlipActive = false;
    _zenithFlipDirection = false;
    
    DEBUG_PRINTF("TrackingEngine: Started tracking %s\n", _tle.name);
    return true;
}

void TrackingEngine::stopTracking() {
    _mode = TrackingMode::IDLE;
    _zenithFlipActive = false;
    stepperControl.stop();
    
    DEBUG_PRINTLN("TrackingEngine: Tracking stopped");
}

void TrackingEngine::getObserverLocation(double& lat, double& lon, double& alt) const {
    lat = _observerLat;
    lon = _observerLon;
    alt = _observerAlt;
}

void TrackingEngine::setObserverLocation(double lat, double lon, double alt) {
    _observerLat = lat;
    _observerLon = lon;
    _observerAlt = alt;
    _useGPSLocation = false;
    
    // Update SGP4 site and cache
    double altKm = alt / 1000.0;
    _sgp4.site(lat, lon, altKm);
    _cachedSiteLat = lat;
    _cachedSiteLon = lon;
    _cachedSiteAlt = altKm;
    _siteInitialized = true;
    
    DEBUG_PRINTF("TrackingEngine: Observer location set: %.6f, %.6f, %.1fm\n",
                 lat, lon, alt);
}

void TrackingEngine::updateObserverLocation() {
    if (_useGPSLocation && gps.hasFix()) {
        _observerLat = gps.getLatitude();
        _observerLon = gps.getLongitude();
        _observerAlt = gps.getAltitude();
    }
}

void TrackingEngine::handleZenithFlip() {
    // Look ahead to predict if we need to flip
    uint32_t futureTime = gps.getUnixTime() + ZENITH_LOOKAHEAD_SEC;
    SatellitePosition futurePos;
    
    if (!calculatePosition(futureTime, futurePos)) return;
    
    // Check if approaching zenith
    if (futurePos.elevation > ZENITH_THRESHOLD_DEG && !_zenithFlipActive) {
        // Calculate azimuth change
        float azChange = futurePos.azimuth - _satPos.azimuth;
        if (azChange > 180) azChange -= 360;
        if (azChange < -180) azChange += 360;
        
        // If large azimuth change expected, activate flip
        if (abs(azChange) > 90) {
            _zenithFlipActive = true;
            _zenithFlipDirection = (azChange > 0);  // Direction of flip
            
            DEBUG_PRINTLN("TrackingEngine: Zenith flip activated");
        }
    }
    
    // Deactivate flip when elevation drops
    if (_zenithFlipActive && _satPos.elevation < ZENITH_THRESHOLD_DEG - 5) {
        _zenithFlipActive = false;
        DEBUG_PRINTLN("TrackingEngine: Zenith flip deactivated");
    }
    
    _lastAzimuth = _satPos.azimuth;
}

float TrackingEngine::getTargetAzimuth() const {
    if (_zenithFlipActive) {
        // Flip azimuth by 180 degrees
        float flippedAz = _satPos.azimuth + 180.0f;
        if (flippedAz >= 360.0f) flippedAz -= 360.0f;
        return flippedAz;
    }
    return _satPos.azimuth;
}

float TrackingEngine::getTargetElevation() const {
    if (_zenithFlipActive) {
        // During zenith flip, azimuth is rotated 180° so elevation counts down from 90°
        // When satellite is at 85° actual elevation, we point to 90° - (90° - 85°) = 85°
        // but from the opposite azimuth direction, effectively "mirroring" over zenith
        // The elevation stays as-is since we're now pointing from the other side
        return _satPos.elevation;
    }
    return _satPos.elevation;
}

bool TrackingEngine::getSunPosition(float& az, float& el) const {
    uint32_t now = gps.getUnixTime();
    if (now == 0) return false;
    
    double jd = julianDateFromUnix(now);
    calculateSunPosition(jd, _observerLat, _observerLon, az, el);
    
    return (el > 0);
}

bool TrackingEngine::getMoonPosition(float& az, float& el) const {
    uint32_t now = gps.getUnixTime();
    if (now == 0) return false;
    
    double jd = julianDateFromUnix(now);
    calculateMoonPosition(jd, _observerLat, _observerLon, az, el);
    
    return (el > 0);
}

double TrackingEngine::julianDateFromUnix(uint32_t unixTime) const {
    return UNIX_TO_JD_OFFSET + (double)unixTime / SECONDS_PER_DAY;
}

void TrackingEngine::calculateSunPosition(double jd, double lat, double lon, 
                                          float& az, float& el) const {
    // Simplified solar position algorithm
    // Based on NOAA Solar Position Calculator
    
    double n = jd - 2451545.0;  // Days since J2000.0
    
    // Mean longitude (degrees)
    double L = fmod(280.46 + 0.9856474 * n, 360.0);
    
    // Mean anomaly (degrees)
    double g = fmod(357.528 + 0.9856003 * n, 360.0);
    double gRad = g * DEG_TO_RAD;
    
    // Ecliptic longitude (degrees)
    double lambda = L + 1.915 * sin(gRad) + 0.020 * sin(2 * gRad);
    double lambdaRad = lambda * DEG_TO_RAD;
    
    // Obliquity of ecliptic (degrees)
    double epsilon = 23.439 - 0.0000004 * n;
    double epsilonRad = epsilon * DEG_TO_RAD;
    
    // Right ascension and declination
    double sinLambda = sin(lambdaRad);
    double cosLambda = cos(lambdaRad);
    double sinEpsilon = sin(epsilonRad);
    double cosEpsilon = cos(epsilonRad);
    
    double ra = atan2(cosEpsilon * sinLambda, cosLambda);
    double dec = asin(sinEpsilon * sinLambda);
    
    // Greenwich Mean Sidereal Time
    double gmst = fmod(280.46061837 + 360.98564736629 * n, 360.0);
    double gmstRad = gmst * DEG_TO_RAD;
    
    // Local hour angle
    double ha = gmstRad + lon * DEG_TO_RAD - ra;
    
    // Convert to azimuth and elevation
    double latRad = lat * DEG_TO_RAD;
    double sinLat = sin(latRad);
    double cosLat = cos(latRad);
    double sinDec = sin(dec);
    double cosDec = cos(dec);
    double cosHa = cos(ha);
    double sinHa = sin(ha);
    
    // Elevation
    double sinEl = sinLat * sinDec + cosLat * cosDec * cosHa;
    el = asin(sinEl) * RAD_TO_DEG;
    
    // Azimuth
    double y = -cosDec * sinHa;
    double x = cosLat * sinDec - sinLat * cosDec * cosHa;
    az = atan2(y, x) * RAD_TO_DEG;
    if (az < 0) az += 360.0;
}

void TrackingEngine::calculateMoonPosition(double jd, double lat, double lon,
                                           float& az, float& el) const {
    // Simplified lunar position algorithm
    // Less accurate than sun, but sufficient for alignment
    
    double n = jd - 2451545.0;  // Days since J2000.0
    
    // Moon's mean longitude (degrees)
    double L = fmod(218.316 + 13.176396 * n, 360.0);
    
    // Moon's mean anomaly (degrees)
    double M = fmod(134.963 + 13.064993 * n, 360.0);
    double MRad = M * DEG_TO_RAD;
    
    // Moon's mean distance from ascending node (degrees)
    double F = fmod(93.272 + 13.229350 * n, 360.0);
    double FRad = F * DEG_TO_RAD;
    
    // Ecliptic longitude
    double lambda = L + 6.289 * sin(MRad);
    double lambdaRad = lambda * DEG_TO_RAD;
    
    // Ecliptic latitude
    double beta = 5.128 * sin(FRad);
    double betaRad = beta * DEG_TO_RAD;
    
    // Obliquity
    double epsilon = 23.439 * DEG_TO_RAD;
    
    // Convert to equatorial coordinates
    double sinLambda = sin(lambdaRad);
    double cosLambda = cos(lambdaRad);
    double sinBeta = sin(betaRad);
    double cosBeta = cos(betaRad);
    double sinEpsilon = sin(epsilon);
    double cosEpsilon = cos(epsilon);
    
    double ra = atan2(sinLambda * cosEpsilon - tan(betaRad) * sinEpsilon, cosLambda);
    double dec = asin(sinBeta * cosEpsilon + cosBeta * sinEpsilon * sinLambda);
    
    // Greenwich Mean Sidereal Time
    double gmst = fmod(280.46061837 + 360.98564736629 * n, 360.0);
    double gmstRad = gmst * DEG_TO_RAD;
    
    // Local hour angle
    double ha = gmstRad + lon * DEG_TO_RAD - ra;
    
    // Convert to azimuth and elevation
    double latRad = lat * DEG_TO_RAD;
    double sinLat = sin(latRad);
    double cosLat = cos(latRad);
    double sinDec = sin(dec);
    double cosDec = cos(dec);
    double cosHa = cos(ha);
    double sinHa = sin(ha);
    
    double sinEl = sinLat * sinDec + cosLat * cosDec * cosHa;
    el = asin(sinEl) * RAD_TO_DEG;
    
    double y = -cosDec * sinHa;
    double x = cosLat * sinDec - sinLat * cosDec * cosHa;
    az = atan2(y, x) * RAD_TO_DEG;
    if (az < 0) az += 360.0;
}

// =========================================================================
// Multi-Pass Prediction Implementation
// =========================================================================

int TrackingEngine::getPasses(PassInfo* passes, int maxPasses, int hoursAhead,
                               float minElevation) {
    if (!_tle.valid) return 0;
    
    uint32_t now = gps.getUnixTime();
    if (now == 0) return 0;
    
    uint32_t searchEnd = now + hoursAhead * 3600;
    uint32_t step = 60;  // 1 minute steps
    
    int passCount = 0;
    bool inPass = false;
    PassInfo currentPass;
    
    for (uint32_t t = now; t < searchEnd && passCount < maxPasses; t += step) {
        SatellitePosition pos;
        if (!calculatePosition(t, pos)) continue;
        
        if (!inPass && pos.elevation > 0) {
            // Found AOS
            inPass = true;
            currentPass.aosTime = t;
            currentPass.aosAz = pos.azimuth;
            currentPass.maxEl = pos.elevation;
            currentPass.maxElTime = t;
            currentPass.maxElAz = pos.azimuth;
            
            // Refine AOS
            for (uint32_t t2 = t - step; t2 < t; t2 += 5) {
                if (calculatePosition(t2, pos) && pos.elevation > 0) {
                    currentPass.aosTime = t2;
                    currentPass.aosAz = pos.azimuth;
                    break;
                }
            }
        } else if (inPass) {
            // Track maximum elevation
            if (pos.elevation > currentPass.maxEl) {
                currentPass.maxEl = pos.elevation;
                currentPass.maxElTime = t;
                currentPass.maxElAz = pos.azimuth;
            }
            
            if (pos.elevation <= 0) {
                // Found LOS
                currentPass.losTime = t;
                currentPass.losAz = pos.azimuth;
                
                // Refine LOS
                for (uint32_t t2 = t - step; t2 < t; t2 += 5) {
                    if (calculatePosition(t2, pos) && pos.elevation <= 0) {
                        currentPass.losTime = t2;
                        currentPass.losAz = pos.azimuth;
                        break;
                    }
                }
                
                // Only add if meets minimum elevation requirement
                if (currentPass.maxEl >= minElevation) {
                    passes[passCount++] = currentPass;
                }
                
                inPass = false;
            }
        }
    }
    
    return passCount;
}

bool TrackingEngine::schedulePass(const char* satellite, const PassInfo& pass, bool prePosition) {
    if (_scheduleCount >= MAX_SCHEDULED_PASSES) {
        return false;
    }
    
    // Find empty slot
    for (int i = 0; i < MAX_SCHEDULED_PASSES; i++) {
        if (!_schedule[i].active) {
            _schedule[i].pass = pass;
            strncpy(_schedule[i].satellite, satellite, sizeof(_schedule[i].satellite) - 1);
            _schedule[i].autoTrack = true;
            _schedule[i].prePosition = prePosition;
            _schedule[i].priority = 0;
            _schedule[i].active = true;
            _scheduleCount++;
            
            DEBUG_PRINTF("TrackingEngine: Scheduled pass for %s at %lu\n", 
                         satellite, pass.aosTime);
            return true;
        }
    }
    
    return false;
}

void TrackingEngine::cancelScheduledPass(int index) {
    if (index >= 0 && index < MAX_SCHEDULED_PASSES && _schedule[index].active) {
        _schedule[index].active = false;
        _scheduleCount--;
    }
}

int TrackingEngine::getScheduledPasses(ScheduledPass* passes, int maxPasses) {
    int count = 0;
    for (int i = 0; i < MAX_SCHEDULED_PASSES && count < maxPasses; i++) {
        if (_schedule[i].active) {
            passes[count++] = _schedule[i];
        }
    }
    return count;
}

void TrackingEngine::clearSchedule() {
    for (int i = 0; i < MAX_SCHEDULED_PASSES; i++) {
        _schedule[i].active = false;
    }
    _scheduleCount = 0;
}

void TrackingEngine::checkScheduledPasses() {
    if (_scheduleCount == 0) return;
    
    uint32_t now = gps.getUnixTime();
    if (now == 0) return;
    
    for (int i = 0; i < MAX_SCHEDULED_PASSES; i++) {
        if (!_schedule[i].active) continue;
        
        ScheduledPass& sp = _schedule[i];
        
        // Check if pass has ended
        if (now > sp.pass.losTime + 60) {
            sp.active = false;
            _scheduleCount--;
            continue;
        }
        
        // Check if it's time to start tracking
        if (sp.autoTrack && now >= sp.pass.aosTime && now <= sp.pass.losTime) {
            // Load TLE and start tracking
            // Note: Caller should ensure TLE is available
            if (startTracking()) {
                DEBUG_PRINTF("TrackingEngine: Auto-started tracking %s\n", sp.satellite);
            }
        }
    }
}

// =========================================================================
// Pre-Positioning Implementation
// =========================================================================

void TrackingEngine::setPrePositioning(bool enable, uint32_t secondsBefore) {
    _prePositionEnabled = enable;
    _prePositionSeconds = secondsBefore;
}

void TrackingEngine::handlePrePositioning() {
    if (!_prePositionEnabled || _mode != TrackingMode::IDLE) {
        _prePositioning = false;
        return;
    }
    
    if (!_tle.valid) return;
    
    uint32_t now = gps.getUnixTime();
    if (now == 0) return;
    
    PassInfo nextPass;
    if (!getNextPass(nextPass)) return;
    
    // Check if we should pre-position
    uint32_t prePositionTime = nextPass.aosTime - _prePositionSeconds;
    
    if (now >= prePositionTime && now < nextPass.aosTime) {
        if (!_prePositioning) {
            _prePositioning = true;
            
            // Move to AOS position at low elevation
            float targetAz = nextPass.aosAz;
            float targetEl = PRE_POSITION_ELEVATION;
            
            // Store original values to avoid using modified az for el calculation
            if (_antennaPattern.enabled) {
                float origAz = targetAz;
                float origEl = targetEl;
                targetAz = applyAntennaPattern(origAz, origEl, true);
                targetEl = applyAntennaPattern(origAz, origEl, false);
            }
            
            stepperControl.moveTo(targetAz, targetEl, MAX_SLEW_SPEED_DEG_S);
            DEBUG_PRINTF("TrackingEngine: Pre-positioning to Az=%.1f for upcoming pass\n", targetAz);
        }
    } else {
        _prePositioning = false;
    }
}

// =========================================================================
// Doppler Calculation Implementation
// =========================================================================

DopplerInfo TrackingEngine::getDoppler(float uplinkMHz, float downlinkMHz) {
    DopplerInfo info;
    memset(&info, 0, sizeof(DopplerInfo));
    
    // Get range rate from current position (already calculated by SGP4)
    info.rangeRateKmS = _satPos.rangeRate;
    
    // Calculate Doppler shift: delta_f = -f * (v/c)
    // Negative because positive range rate means receding (red shift)
    
    if (downlinkMHz > 0) {
        info.rxShiftHz = -downlinkMHz * 1e6 * (info.rangeRateKmS / SPEED_OF_LIGHT_KMS);
        info.correctedRxMHz = downlinkMHz + (info.rxShiftHz / 1e6);
    }
    
    if (uplinkMHz > 0) {
        info.txShiftHz = -uplinkMHz * 1e6 * (info.rangeRateKmS / SPEED_OF_LIGHT_KMS);
        info.correctedTxMHz = uplinkMHz - (info.txShiftHz / 1e6);  // Opposite correction for TX
    }
    
    return info;
}

// =========================================================================
// Antenna Pattern Compensation Implementation
// =========================================================================

void TrackingEngine::setAntennaPattern(const AntennaPattern& pattern) {
    _antennaPattern = pattern;
    DEBUG_PRINTF("TrackingEngine: Antenna pattern set: Az=%.2f, El=%.2f, BW=%.1f\n",
                 pattern.azBeamOffset, pattern.elBeamOffset, pattern.beamwidth);
}

float TrackingEngine::applyAntennaPattern(float az, float el, bool isAzimuth) const {
    if (!_antennaPattern.enabled) {
        return isAzimuth ? az : el;
    }
    
    if (isAzimuth) {
        return az - _antennaPattern.azBeamOffset;
    } else {
        return el - _antennaPattern.elBeamOffset;
    }
}

// =========================================================================
// EME Mode Implementation
// =========================================================================

bool TrackingEngine::enableEMEMode() {
    float moonAz, moonEl;
    if (!getMoonPosition(moonAz, moonEl)) {
        DEBUG_PRINTLN("TrackingEngine: Cannot enable EME - moon below horizon");
        return false;
    }
    
    _mode = TrackingMode::EME;
    _zenithFlipActive = false;
    
    DEBUG_PRINTLN("TrackingEngine: EME mode enabled");
    return true;
}

void TrackingEngine::disableEMEMode() {
    if (_mode == TrackingMode::EME) {
        _mode = TrackingMode::IDLE;
        DEBUG_PRINTLN("TrackingEngine: EME mode disabled");
    }
}

void TrackingEngine::calculateMoonPositionHighPrecision(double jd, double lat, double lon,
                                                         float& az, float& el, float& distance) const {
    // Higher precision lunar position calculation for EME
    // Based on simplified lunar theory
    
    double T = (jd - 2451545.0) / 36525.0;  // Julian centuries from J2000
    
    // Moon's mean longitude
    double Lp = fmod(218.3164477 + 481267.88123421 * T 
                     - 0.0015786 * T * T, 360.0);
    
    // Moon's mean elongation
    double D = fmod(297.8501921 + 445267.1114034 * T 
                    - 0.0018819 * T * T, 360.0);
    
    // Sun's mean anomaly
    double M = fmod(357.5291092 + 35999.0502909 * T, 360.0);
    
    // Moon's mean anomaly
    double Mp = fmod(134.9633964 + 477198.8675055 * T 
                     + 0.0087414 * T * T, 360.0);
    
    // Moon's argument of latitude
    double F = fmod(93.2720950 + 483202.0175233 * T, 360.0);
    
    // Convert to radians
    double LpRad = Lp * DEG_TO_RAD;
    double DRad = D * DEG_TO_RAD;
    double MRad = M * DEG_TO_RAD;
    double MpRad = Mp * DEG_TO_RAD;
    double FRad = F * DEG_TO_RAD;
    
    // Periodic terms for longitude (simplified)
    double dL = 6288774 * sin(MpRad)
              + 1274027 * sin(2*DRad - MpRad)
              + 658314 * sin(2*DRad)
              + 213618 * sin(2*MpRad)
              - 185116 * sin(MRad)
              - 114332 * sin(2*FRad);
    
    // Periodic terms for distance (simplified)
    double dR = -20905355 * cos(MpRad)
              - 3699111 * cos(2*DRad - MpRad)
              - 2955968 * cos(2*DRad)
              - 569925 * cos(2*MpRad);
    
    // Periodic terms for latitude (simplified)
    double dB = 5128122 * sin(FRad)
              + 280602 * sin(MpRad + FRad)
              + 277693 * sin(MpRad - FRad)
              + 173237 * sin(2*DRad - FRad);
    
    // Apply corrections
    double lambda = Lp + dL / 1000000.0;  // Ecliptic longitude
    double beta = dB / 1000000.0;          // Ecliptic latitude
    distance = 385000.56 + dR / 1000.0;    // Distance in km
    
    // Convert ecliptic to equatorial coordinates
    double lambdaRad = lambda * DEG_TO_RAD;
    double betaRad = beta * DEG_TO_RAD;
    double epsilon = (23.439 - 0.00000036 * (jd - 2451545.0)) * DEG_TO_RAD;
    
    double sinLambda = sin(lambdaRad);
    double cosLambda = cos(lambdaRad);
    double sinBeta = sin(betaRad);
    double cosBeta = cos(betaRad);
    double sinEpsilon = sin(epsilon);
    double cosEpsilon = cos(epsilon);
    
    double ra = atan2(sinLambda * cosEpsilon - tan(betaRad) * sinEpsilon, cosLambda);
    double dec = asin(sinBeta * cosEpsilon + cosBeta * sinEpsilon * sinLambda);
    
    // Convert to horizontal coordinates
    double gmst = fmod(280.46061837 + 360.98564736629 * (jd - 2451545.0), 360.0);
    double ha = (gmst + lon) * DEG_TO_RAD - ra;
    
    double latRad = lat * DEG_TO_RAD;
    double sinLat = sin(latRad);
    double cosLat = cos(latRad);
    double sinDec = sin(dec);
    double cosDec = cos(dec);
    double cosHa = cos(ha);
    double sinHa = sin(ha);
    
    double sinEl = sinLat * sinDec + cosLat * cosDec * cosHa;
    el = asin(sinEl) * RAD_TO_DEG;
    
    double y = -cosDec * sinHa;
    double x = cosLat * sinDec - sinLat * cosDec * cosHa;
    az = atan2(y, x) * RAD_TO_DEG;
    if (az < 0) az += 360.0;
}

float TrackingEngine::getEMEPathLoss(float freqMHz) const {
    // EME path loss calculation
    // Path loss = 32.45 + 20*log10(f_MHz) + 20*log10(d_km) + surface_reflection_loss
    // For EME: distance = 2 * moon_distance, reflection loss ≈ 6-7 dB
    
    float totalDistance = 2 * _moonDistance;  // Round trip
    float pathLoss = 32.45 + 20 * log10(freqMHz) + 20 * log10(totalDistance);
    float reflectionLoss = 6.5;  // Average lunar surface reflection loss
    
    return pathLoss + reflectionLoss;
}

// =========================================================================
// Geostationary Mode Implementation
// =========================================================================

bool TrackingEngine::calculateGeoPosition(float geoLongitude, float& az, float& el) {
    // Calculate look angles to geostationary satellite
    // Using standard formulas for geostationary orbit
    
    double latRad = _observerLat * DEG_TO_RAD;
    double lonDiff = (geoLongitude - _observerLon) * DEG_TO_RAD;
    
    // Earth radius and geo orbit radius
    double Re = 6378.137;  // km
    double Rs = Re + GEO_ALTITUDE_KM;
    
    // Calculate elevation angle
    double cosLat = cos(latRad);
    double sinLat = sin(latRad);
    double cosLonDiff = cos(lonDiff);
    double sinLonDiff = sin(lonDiff);
    
    double d = sqrt(1 + pow(Rs/Re, 2) - 2 * (Rs/Re) * cosLat * cosLonDiff);
    double sinEl = ((Rs/Re) * cosLat * cosLonDiff - 1) / d;
    
    el = asin(sinEl) * RAD_TO_DEG;
    
    // Check if visible (elevation > 0)
    if (el < 0) {
        return false;
    }
    
    // Calculate azimuth using standard geostationary formula
    // az = 180 + atan2(tan(lonDiff), sin(lat))
    // This gives azimuth measured clockwise from North
    az = 180.0 + atan2(tan(lonDiff), sinLat) * RAD_TO_DEG;
    
    // Normalize azimuth to 0-360
    while (az < 0) az += 360.0;
    while (az >= 360.0) az -= 360.0;
    
    return true;
}

bool TrackingEngine::pointToGeo(float longitude, const char* name) {
    float az, el;
    
    if (!calculateGeoPosition(longitude, az, el)) {
        DEBUG_PRINTF("TrackingEngine: Geo position %.1f not visible\n", longitude);
        return false;
    }
    
    // Apply antenna pattern compensation
    // Store original values to avoid using modified az for el calculation
    if (_antennaPattern.enabled) {
        float origAz = az;
        float origEl = el;
        az = applyAntennaPattern(origAz, origEl, true);
        el = applyAntennaPattern(origAz, origEl, false);
    }
    
    stepperControl.moveTo(az, el, MAX_SLEW_SPEED_DEG_S);
    
    DEBUG_PRINTF("TrackingEngine: Pointing to geo %.1f° -> Az=%.1f, El=%.1f\n",
                 longitude, az, el);
    return true;
}

bool TrackingEngine::enableGeoTracking(const GeoSatConfig& config) {
    if (!calculateGeoPosition(config.longitude, _satPos.azimuth, _satPos.elevation)) {
        return false;
    }
    
    _geoConfig = config;
    _geoConfig.active = true;
    _mode = TrackingMode::GEO;
    _lastGeoUpdate = 0;  // Force immediate update
    
    DEBUG_PRINTF("TrackingEngine: Geo tracking enabled for %s at %.1f°\n",
                 config.name, config.longitude);
    return true;
}

void TrackingEngine::disableGeoTracking() {
    if (_mode == TrackingMode::GEO) {
        _geoConfig.active = false;
        _mode = TrackingMode::IDLE;
        DEBUG_PRINTLN("TrackingEngine: Geo tracking disabled");
    }
}
