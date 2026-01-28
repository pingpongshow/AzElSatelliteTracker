/**
 * @file gps.cpp
 * @brief GPS NMEA parsing implementation
 */

#include "gps.h"

// Global instance
GPS gps;

GPS::GPS() 
    : _serial(nullptr)
    , _useManualPosition(false)
    , _manualLat(DEFAULT_LATITUDE)
    , _manualLon(DEFAULT_LONGITUDE)
    , _manualAlt(DEFAULT_ALTITUDE)
    , _lastUpdateMs(0)
    , _lastFixMs(0)
{
    // Initialize data structure with defaults
    memset(&_data, 0, sizeof(GPSData));
    _data.latitude = DEFAULT_LATITUDE;
    _data.longitude = DEFAULT_LONGITUDE;
    _data.altitude = DEFAULT_ALTITUDE;
    _data.fixStatus = GPSFixStatus::NO_FIX;
    _data.valid = false;
}

bool GPS::begin() {
    DEBUG_PRINTLN("GPS: Initializing...");
    
    // Initialize hardware serial
    _serial = new HardwareSerial(GPS_SERIAL_NUM);
    _serial->begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    
    _lastUpdateMs = millis();
    
    DEBUG_PRINTF("GPS: Serial initialized on pins RX=%d, TX=%d at %d baud\n",
                 GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
    
    return true;
}

void GPS::update() {
    // Read all available data from GPS
    while (_serial->available() > 0) {
        char c = _serial->read();
        if (_gps.encode(c)) {
            parseData();
        }
    }
    
    // Check for stale data
    uint32_t now = millis();
    if (_data.valid && (now - _lastFixMs > GPS_UPDATE_INTERVAL_MS * 5)) {
        _data.valid = false;
        _data.fixStatus = GPSFixStatus::NO_FIX;
        DEBUG_PRINTLN("GPS: Fix lost (stale data)");
    }
    
    _lastUpdateMs = now;
}

void GPS::parseData() {
    // Update position if valid
    if (_gps.location.isValid() && !_useManualPosition) {
        _data.latitude = _gps.location.lat();
        _data.longitude = _gps.location.lng();
        _data.valid = true;
        _lastFixMs = millis();
        
        // Determine fix type based on available data
        if (_gps.altitude.isValid()) {
            _data.fixStatus = GPSFixStatus::FIX_3D;
            _data.altitude = _gps.altitude.meters();
        } else {
            _data.fixStatus = GPSFixStatus::FIX_2D;
        }
    } else if (_useManualPosition) {
        _data.latitude = _manualLat;
        _data.longitude = _manualLon;
        _data.altitude = _manualAlt;
        _data.valid = true;
        _data.fixStatus = GPSFixStatus::FIX_3D;
    }
    
    // Update altitude
    if (_gps.altitude.isValid() && !_useManualPosition) {
        _data.altitude = _gps.altitude.meters();
    }
    
    // Update time
    if (_gps.date.isValid() && _gps.time.isValid()) {
        _data.year = _gps.date.year();
        _data.month = _gps.date.month();
        _data.day = _gps.date.day();
        _data.hour = _gps.time.hour();
        _data.minute = _gps.time.minute();
        _data.second = _gps.time.second();
        _data.centisecond = _gps.time.centisecond();
        
        _data.unixTime = dateTimeToUnix(_data.year, _data.month, _data.day,
                                        _data.hour, _data.minute, _data.second);
    }
    
    // Update satellite info
    if (_gps.satellites.isValid()) {
        _data.satellites = _gps.satellites.value();
    }
    
    // Update HDOP
    if (_gps.hdop.isValid()) {
        _data.hdop = _gps.hdop.hdop();
    }
    
    // Update speed and course
    if (_gps.speed.isValid()) {
        _data.speed = _gps.speed.mps();
    }
    if (_gps.course.isValid()) {
        _data.course = _gps.course.deg();
    }
    
    // Update age
    _data.age = _gps.location.age();
}

double GPS::getJulianDate() const {
    // Calculate Julian Date from GPS time
    // JD = 367*Y - INT(7*(Y+INT((M+9)/12))/4) + INT(275*M/9) + D + 1721013.5 + UT/24
    
    int y = _data.year;
    int m = _data.month;
    int d = _data.day;
    double ut = _data.hour + _data.minute / 60.0 + _data.second / 3600.0;
    
    double jd = 367.0 * y - floor(7.0 * (y + floor((m + 9.0) / 12.0)) / 4.0)
                + floor(275.0 * m / 9.0) + d + 1721013.5 + ut / 24.0;
    
    return jd;
}

double GPS::getGPSTime() const {
    // Calculate time since GPS epoch (Jan 6, 1980 00:00:00 UTC)
    // Returns minutes since epoch for TLE calculations
    
    // GPS epoch in Unix time
    const uint32_t GPS_EPOCH_UNIX = 315964800;  // Jan 6, 1980
    
    if (_data.unixTime > GPS_EPOCH_UNIX) {
        return (_data.unixTime - GPS_EPOCH_UNIX) / 60.0;
    }
    
    return 0;
}

bool GPS::hasValidTime() const {
    // Check TinyGPS++ validity flags
    if (!_gps.time.isValid() || !_gps.date.isValid()) {
        return false;
    }
    
    // Validate year is reasonable (after 2020)
    if (_data.year < 2020) {
        return false;
    }
    
    // Validate month range (1-12)
    if (_data.month < 1 || _data.month > 12) {
        return false;
    }
    
    // Validate day range based on month
    static const uint8_t daysInMonth[] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    uint8_t maxDay = daysInMonth[_data.month - 1];
    
    // Adjust February for non-leap years
    if (_data.month == 2) {
        bool isLeapYear = (_data.year % 4 == 0 && (_data.year % 100 != 0 || _data.year % 400 == 0));
        if (!isLeapYear) {
            maxDay = 28;
        }
    }
    
    if (_data.day < 1 || _data.day > maxDay) {
        return false;
    }
    
    // Validate time ranges
    if (_data.hour > 23 || _data.minute > 59 || _data.second > 60) {  // 60 allows for leap seconds
        return false;
    }
    
    return true;
}

void GPS::setManualPosition(double lat, double lon, double alt) {
    _manualLat = lat;
    _manualLon = lon;
    _manualAlt = alt;
    
    if (_useManualPosition) {
        _data.latitude = lat;
        _data.longitude = lon;
        _data.altitude = alt;
        _data.valid = true;
        _data.fixStatus = GPSFixStatus::FIX_3D;
    }
    
    DEBUG_PRINTF("GPS: Manual position set: %.6f, %.6f, %.1fm\n", lat, lon, alt);
}

String GPS::getStatusString() const {
    String status = "GPS: ";
    
    if (!_data.valid) {
        status += "No Fix";
    } else {
        status += (_data.fixStatus == GPSFixStatus::FIX_3D) ? "3D Fix" : "2D Fix";
        status += " | Lat: " + String(_data.latitude, 6);
        status += " | Lon: " + String(_data.longitude, 6);
        status += " | Alt: " + String(_data.altitude, 1) + "m";
        status += " | Sats: " + String(_data.satellites);
    }
    
    if (_useManualPosition) {
        status += " [MANUAL]";
    }
    
    return status;
}

uint32_t GPS::dateTimeToUnix(uint16_t year, uint8_t month, uint8_t day,
                              uint8_t hour, uint8_t minute, uint8_t second) {
    // Days in each month (non-leap year)
    static const uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    
    // Calculate days since 1970
    uint32_t days = 0;
    
    // Full years
    for (uint16_t y = 1970; y < year; y++) {
        days += 365;
        if ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0)) {
            days++;  // Leap year
        }
    }
    
    // Full months this year
    for (uint8_t m = 1; m < month; m++) {
        days += daysInMonth[m - 1];
        if (m == 2 && ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))) {
            days++;  // Leap year February
        }
    }
    
    // Days this month
    days += day - 1;
    
    // Convert to seconds
    uint32_t unix = days * 86400UL;
    unix += hour * 3600UL;
    unix += minute * 60UL;
    unix += second;
    
    return unix;
}
