/**
 * @file gps.h
 * @brief GPS NMEA parsing and position management
 * 
 * Provides GPS position and time for satellite tracking calculations.
 */

#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <TinyGPS++.h>
#include "config.h"

// GPS fix status
enum class GPSFixStatus {
    NO_FIX,
    FIX_2D,
    FIX_3D
};

// GPS data structure
struct GPSData {
    double latitude;        // Degrees (positive = North)
    double longitude;       // Degrees (positive = East)
    double altitude;        // Meters above sea level
    float speed;            // Speed in m/s
    float course;           // Course in degrees
    uint8_t satellites;     // Number of satellites
    float hdop;             // Horizontal dilution of precision
    GPSFixStatus fixStatus;
    bool valid;             // Data validity flag
    
    // Time (UTC)
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint32_t centisecond;
    
    // Unix timestamp
    uint32_t unixTime;
    
    // Age of data in milliseconds
    uint32_t age;
};

/**
 * @class GPS
 * @brief GPS module interface using TinyGPS++
 */
class GPS {
public:
    GPS();
    
    /**
     * @brief Initialize GPS serial port
     * @return true if initialization successful
     */
    bool begin();
    
    /**
     * @brief Process incoming GPS data (call frequently)
     */
    void update();
    
    /**
     * @brief Get current GPS data
     * @return GPS data structure
     */
    GPSData getData() const { return _data; }
    
    /**
     * @brief Check if GPS has valid fix
     * @return true if valid fix available
     */
    bool hasFix() const { return _data.valid && _data.fixStatus != GPSFixStatus::NO_FIX; }
    
    /**
     * @brief Get latitude
     * @return Latitude in degrees (positive = North)
     */
    double getLatitude() const { return _data.latitude; }
    
    /**
     * @brief Get longitude
     * @return Longitude in degrees (positive = East)
     */
    double getLongitude() const { return _data.longitude; }
    
    /**
     * @brief Get altitude
     * @return Altitude in meters
     */
    double getAltitude() const { return _data.altitude; }
    
    /**
     * @brief Get number of satellites
     */
    uint8_t getSatellites() const { return _data.satellites; }
    
    /**
     * @brief Get Unix timestamp
     * @return Seconds since Jan 1, 1970
     */
    uint32_t getUnixTime() const { return _data.unixTime; }
    
    /**
     * @brief Get Julian date for satellite calculations
     * @return Julian date
     */
    double getJulianDate() const;
    
    /**
     * @brief Get time since GPS epoch for TLE calculations
     * @return Minutes since GPS epoch (Jan 6, 1980)
     */
    double getGPSTime() const;
    
    /**
     * @brief Check if time is valid
     * @return true if GPS time is valid
     */
    bool hasValidTime() const;
    
    /**
     * @brief Set manual position (override GPS)
     * @param lat Latitude in degrees
     * @param lon Longitude in degrees
     * @param alt Altitude in meters
     */
    void setManualPosition(double lat, double lon, double alt);
    
    /**
     * @brief Use manual position instead of GPS
     * @param useManual true to use manual position
     */
    void setUseManualPosition(bool useManual) { _useManualPosition = useManual; }
    
    /**
     * @brief Check if using manual position
     */
    bool isUsingManualPosition() const { return _useManualPosition; }
    
    /**
     * @brief Get raw TinyGPS++ object (for advanced use)
     */
    TinyGPSPlus& getRaw() { return _gps; }
    
    /**
     * @brief Get statistics string
     */
    String getStatusString() const;

private:
    TinyGPSPlus _gps;
    HardwareSerial* _serial;
    GPSData _data;
    
    // Manual position override
    bool _useManualPosition;
    double _manualLat;
    double _manualLon;
    double _manualAlt;
    
    // Timestamps
    uint32_t _lastUpdateMs;
    uint32_t _lastFixMs;
    
    // Internal methods
    void parseData();
    uint32_t dateTimeToUnix(uint16_t year, uint8_t month, uint8_t day,
                            uint8_t hour, uint8_t minute, uint8_t second);
};

// Global instance
extern GPS gps;

#endif // GPS_H
