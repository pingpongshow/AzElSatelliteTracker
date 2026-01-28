/**
 * @file web_handlers_tracking.cpp
 * @brief Web server handlers for satellite tracking functionality
 * 
 * This file contains WebServer method implementations for:
 * - Satellite tracking (start/stop)
 * - TLE management (get, load, fetch, delete)
 * - Pass prediction and scheduling
 * - Doppler shift calculations
 * - External API integration (N2YO, SatNOGS, Celestrak)
 * - EME (Earth-Moon-Earth) mode
 * - Geostationary satellite tracking
 * - Antenna pattern configuration
 * - Pre-positioning settings
 * 
 * Separated from web_server.cpp for better maintainability.
 */

#include "web_server.h"
#include "tracking_engine.h"
#include "tle_manager.h"
#include "gps.h"
#include "external_api.h"
#include "wifi_manager.h"

// =============================================================================
// TRACKING HANDLERS
// =============================================================================

void WebServer::handleStartTracking(AsyncWebServerRequest* request, JsonVariant& json) {
    if (!json.containsKey("name")) {
        sendError(request, "Missing satellite name");
        return;
    }
    
    String name = json["name"].as<String>();
    
    // Check if TLE provided or should load from storage
    if (json.containsKey("line1") && json.containsKey("line2")) {
        String line1 = json["line1"].as<String>();
        String line2 = json["line2"].as<String>();
        
        if (!trackingEngine.loadTLE(name.c_str(), line1.c_str(), line2.c_str())) {
            sendError(request, "Invalid TLE data");
            return;
        }
    } else {
        // Try to load from storage
        TLEEntry entry;
        if (!tleManager.findStoredTLE(name.c_str(), entry)) {
            sendError(request, "TLE not found in storage");
            return;
        }
        
        if (!trackingEngine.loadTLE(entry.name, entry.line1, entry.line2)) {
            sendError(request, "Failed to load TLE");
            return;
        }
    }
    
    if (trackingEngine.startTracking()) {
        sendOK(request, "Tracking started");
    } else {
        sendError(request, "Failed to start tracking");
    }
}

void WebServer::handleStopTracking(AsyncWebServerRequest* request) {
    trackingEngine.stopTracking();
    sendOK(request, "Tracking stopped");
}

void WebServer::handleGetNextPass(AsyncWebServerRequest* request) {
    PassInfo pass;
    
    if (!trackingEngine.getNextPass(pass)) {
        sendError(request, "No pass found", 404);
        return;
    }
    
    StaticJsonDocument<JSON_SIZE_MEDIUM> doc;
    doc["aosTime"] = pass.aosTime;
    doc["losTime"] = pass.losTime;
    doc["maxElTime"] = pass.maxElTime;
    doc["aosAz"] = pass.aosAz;
    doc["losAz"] = pass.losAz;
    doc["maxEl"] = pass.maxEl;
    doc["maxElAz"] = pass.maxElAz;
    doc["duration"] = pass.losTime - pass.aosTime;
    
    sendJSON(request, doc);
}

// =============================================================================
// TLE HANDLERS
// =============================================================================

void WebServer::handleGetTLEs(AsyncWebServerRequest* request) {
    StaticJsonDocument<JSON_SIZE_XLARGE> doc;
    JsonArray tles = doc["tles"].to<JsonArray>();
    
    String names[MAX_STORED_TLES];
    int count = tleManager.listStoredTLEs(names, MAX_STORED_TLES);
    
    for (int i = 0; i < count; i++) {
        TLEEntry entry;
        if (tleManager.findStoredTLE(names[i].c_str(), entry)) {
            JsonObject tle = tles.createNestedObject();
            tle["name"] = entry.name;
            tle["stale"] = tleManager.isTLEStale(entry);
        }
    }
    
    doc["count"] = count;
    sendJSON(request, doc);
}

void WebServer::handleLoadTLE(AsyncWebServerRequest* request, JsonVariant& json) {
    if (!json.containsKey("name") || !json.containsKey("line1") || !json.containsKey("line2")) {
        sendError(request, "Missing TLE data");
        return;
    }
    
    TLEEntry entry;
    strncpy(entry.name, json["name"].as<const char*>(), sizeof(entry.name) - 1);
    strncpy(entry.line1, json["line1"].as<const char*>(), sizeof(entry.line1) - 1);
    strncpy(entry.line2, json["line2"].as<const char*>(), sizeof(entry.line2) - 1);
    entry.fetchTime = millis() / 1000;
    entry.valid = true;
    
    // Find slot using O(1) bitmap lookup
    // First check if satellite with same name already exists
    int slot = tleManager.findSlotByName(entry.name);
    
    if (slot < 0) {
        // Not found - get first empty slot (O(1) lookup)
        slot = tleManager.findEmptySlot();
    }
    
    if (slot < 0) {
        sendError(request, "No storage space available");
        return;
    }
    
    if (tleManager.saveTLE(entry, slot)) {
        sendOK(request, "TLE saved");
    } else {
        sendError(request, "Failed to save TLE");
    }
}

void WebServer::handleFetchTLE(AsyncWebServerRequest* request, JsonVariant& json) {
    if (!wifiManager.isConnected()) {
        sendError(request, "No network connection");
        return;
    }
    
    TLEEntry entry;
    bool success = false;
    
    if (json.containsKey("noradId")) {
        success = tleManager.fetchByNoradId(json["noradId"].as<uint32_t>(), entry);
    } else if (json.containsKey("name")) {
        success = tleManager.fetchByName(json["name"].as<const char*>(), entry);
    } else {
        sendError(request, "Provide noradId or name");
        return;
    }
    
    if (!success) {
        sendError(request, tleManager.getLastError().c_str());
        return;
    }
    
    // Find appropriate slot using O(1) bitmap lookup
    // First check if TLE with same name already exists (update case)
    int slot = tleManager.findSlotByName(entry.name);
    
    if (slot < 0) {
        // Not found - get first empty slot (O(1) lookup)
        slot = tleManager.findEmptySlot();
    }
    
    if (slot < 0) {
        sendError(request, "No storage space available");
        return;
    }
    
    if (!tleManager.saveTLE(entry, slot)) {
        sendError(request, "Failed to save TLE");
        return;
    }
    
    StaticJsonDocument<JSON_SIZE_LARGE> doc;
    doc["name"] = entry.name;
    doc["line1"] = entry.line1;
    doc["line2"] = entry.line2;
    doc["slot"] = slot;
    sendJSON(request, doc);
}

void WebServer::handleDeleteTLE(AsyncWebServerRequest* request, JsonVariant& json) {
    if (!json.containsKey("name")) {
        sendError(request, "Missing TLE name");
        return;
    }
    
    const char* name = json["name"].as<const char*>();
    
    if (tleManager.deleteTLE(name)) {
        sendOK(request, "TLE deleted");
    } else {
        sendError(request, "TLE not found");
    }
}

// =============================================================================
// PASS PREDICTION HANDLERS
// =============================================================================

void WebServer::handleGetPasses(AsyncWebServerRequest* request) {
    if (!trackingEngine.hasTLE()) {
        sendError(request, "No TLE loaded", 400);
        return;
    }
    
    int hours = request->hasParam("hours") ?
                 request->getParam("hours")->value().toInt() : PASS_SEARCH_HOURS;
    float minEl = request->hasParam("minEl") ?
                   request->getParam("minEl")->value().toFloat() : PASS_MIN_ELEVATION;
    
    PassInfo passes[MAX_PREDICTED_PASSES];
    int count = trackingEngine.getPasses(passes, MAX_PREDICTED_PASSES, hours, minEl);
    
    StaticJsonDocument<JSON_SIZE_XLARGE> doc;
    JsonArray passArray = doc["passes"].to<JsonArray>();
    
    for (int i = 0; i < count; i++) {
        JsonObject p = passArray.createNestedObject();
        p["aosTime"] = passes[i].aosTime;
        p["losTime"] = passes[i].losTime;
        p["maxElTime"] = passes[i].maxElTime;
        p["aosAz"] = passes[i].aosAz;
        p["losAz"] = passes[i].losAz;
        p["maxEl"] = passes[i].maxEl;
        p["maxElAz"] = passes[i].maxElAz;
        p["duration"] = passes[i].losTime - passes[i].aosTime;
    }
    
    doc["count"] = count;
    doc["satellite"] = trackingEngine.getSatelliteName();
    
    sendJSON(request, doc);
}

void WebServer::handleSchedulePass(AsyncWebServerRequest* request, JsonVariant& json) {
    if (!json.containsKey("satellite") || !json.containsKey("aosTime")) {
        sendError(request, "Missing satellite or aosTime", 400);
        return;
    }
    
    PassInfo pass;
    pass.aosTime = json["aosTime"].as<uint32_t>();
    pass.losTime = json["losTime"] | (pass.aosTime + 600);
    pass.maxElTime = json["maxElTime"] | (pass.aosTime + 300);
    pass.aosAz = json["aosAz"] | 0;
    pass.losAz = json["losAz"] | 0;
    pass.maxEl = json["maxEl"] | 45;
    pass.maxElAz = json["maxElAz"] | 180;
    
    bool prePosition = json["prePosition"] | true;
    
    if (trackingEngine.schedulePass(json["satellite"].as<const char*>(), pass, prePosition)) {
        sendOK(request, "Pass scheduled");
    } else {
        sendError(request, "Failed to schedule pass - schedule full", 400);
    }
}

void WebServer::handleCancelPass(AsyncWebServerRequest* request, JsonVariant& json) {
    int index = json["index"] | -1;
    if (index < 0) {
        sendError(request, "Missing index", 400);
        return;
    }
    
    trackingEngine.cancelScheduledPass(index);
    sendOK(request, "Pass cancelled");
}

void WebServer::handleGetSchedule(AsyncWebServerRequest* request) {
    ScheduledPass passes[MAX_SCHEDULED_PASSES];
    int count = trackingEngine.getScheduledPasses(passes, MAX_SCHEDULED_PASSES);
    
    StaticJsonDocument<JSON_SIZE_XLARGE> doc;
    JsonArray passArray = doc["schedule"].to<JsonArray>();
    
    for (int i = 0; i < count; i++) {
        JsonObject p = passArray.createNestedObject();
        p["satellite"] = passes[i].satellite;
        p["aosTime"] = passes[i].pass.aosTime;
        p["losTime"] = passes[i].pass.losTime;
        p["maxEl"] = passes[i].pass.maxEl;
        p["autoTrack"] = passes[i].autoTrack;
        p["prePosition"] = passes[i].prePosition;
        p["active"] = passes[i].active;
    }
    
    doc["count"] = count;
    
    sendJSON(request, doc);
}

void WebServer::handleClearSchedule(AsyncWebServerRequest* request) {
    trackingEngine.clearSchedule();
    sendOK(request, "Schedule cleared");
}

// =============================================================================
// DOPPLER HANDLER
// =============================================================================

void WebServer::handleGetDoppler(AsyncWebServerRequest* request) {
    if (!trackingEngine.hasTLE()) {
        sendError(request, "No TLE loaded", 400);
        return;
    }
    
    float uplink = request->hasParam("uplink") ?
                    request->getParam("uplink")->value().toFloat() : 0;
    float downlink = request->hasParam("downlink") ?
                      request->getParam("downlink")->value().toFloat() : 0;
    
    DopplerInfo doppler = trackingEngine.getDoppler(uplink, downlink);
    
    StaticJsonDocument<JSON_SIZE_SMALL> doc;
    doc["rangeRate"] = doppler.rangeRateKmS;
    doc["rxShift"] = doppler.rxShiftHz;
    doc["txShift"] = doppler.txShiftHz;
    doc["correctedRx"] = doppler.correctedRxMHz;
    doc["correctedTx"] = doppler.correctedTxMHz;
    
    sendJSON(request, doc);
}

// =============================================================================
// EXTERNAL API HANDLERS
// =============================================================================

void WebServer::handleSetN2YOKey(AsyncWebServerRequest* request, JsonVariant& json) {
    const char* key = json["key"];
    if (!key || strlen(key) == 0) {
        sendError(request, "Missing API key", 400);
        return;
    }
    
    externalAPI.setN2YOApiKey(key);
    sendOK(request, "N2YO API key set");
}

void WebServer::handleFetchN2YOTLE(AsyncWebServerRequest* request, JsonVariant& json) {
    if (!externalAPI.hasN2YOApiKey()) {
        sendError(request, "N2YO API key not set", 400);
        return;
    }
    
    uint32_t noradId = json["noradId"] | 0;
    if (noradId == 0) {
        sendError(request, "Missing NORAD ID", 400);
        return;
    }
    
    TLEEntry entry;
    if (externalAPI.fetchTLEFromN2YO(noradId, entry)) {
        StaticJsonDocument<JSON_SIZE_LARGE> doc;
        doc["name"] = entry.name;
        doc["line1"] = entry.line1;
        doc["line2"] = entry.line2;
        sendJSON(request, doc);
    } else {
        sendError(request, externalAPI.getLastError().c_str(), 500);
    }
}

void WebServer::handleGetVisualPasses(AsyncWebServerRequest* request) {
    if (!externalAPI.hasN2YOApiKey()) {
        sendError(request, "N2YO API key not set", 400);
        return;
    }
    
    if (!request->hasParam("noradId")) {
        sendError(request, "Missing noradId parameter", 400);
        return;
    }
    
    uint32_t noradId = request->getParam("noradId")->value().toInt();
    int days = request->hasParam("days") ? request->getParam("days")->value().toInt() : 7;
    int minVis = request->hasParam("minVis") ? request->getParam("minVis")->value().toInt() : 60;
    
    // Get observer location
    GPSData gpsData = gps.getData();
    float lat = gpsData.latitude;
    float lon = gpsData.longitude;
    float alt = gpsData.altitude;
    
    VisualPass passes[10];
    int count = externalAPI.getVisualPasses(noradId, lat, lon, alt, days, minVis, passes, 10);
    
    StaticJsonDocument<JSON_SIZE_XLARGE> doc;
    JsonArray passArray = doc["passes"].to<JsonArray>();
    
    for (int i = 0; i < count; i++) {
        JsonObject p = passArray.createNestedObject();
        p["startTime"] = passes[i].startTime;
        p["maxTime"] = passes[i].maxTime;
        p["endTime"] = passes[i].endTime;
        p["startAz"] = passes[i].startAz;
        p["startEl"] = passes[i].startEl;
        p["maxAz"] = passes[i].maxAz;
        p["maxEl"] = passes[i].maxEl;
        p["endAz"] = passes[i].endAz;
        p["endEl"] = passes[i].endEl;
        p["magnitude"] = passes[i].magnitude;
        p["duration"] = passes[i].duration;
    }
    
    doc["count"] = count;
    sendJSON(request, doc);
}

void WebServer::handleSearchSatNOGS(AsyncWebServerRequest* request) {
    if (!request->hasParam("query")) {
        sendError(request, "Missing query parameter", 400);
        return;
    }
    
    String query = request->getParam("query")->value();
    
    SatNOGSSatellite results[10];
    int count = externalAPI.searchSatNOGS(query.c_str(), results, 10);
    
    StaticJsonDocument<JSON_SIZE_XLARGE> doc;
    JsonArray satArray = doc["satellites"].to<JsonArray>();
    
    for (int i = 0; i < count; i++) {
        JsonObject s = satArray.createNestedObject();
        s["noradId"] = results[i].noradId;
        s["name"] = results[i].name;
        s["status"] = results[i].status;
    }
    
    doc["count"] = count;
    
    sendJSON(request, doc);
}

void WebServer::handleGetTransmitters(AsyncWebServerRequest* request) {
    if (!request->hasParam("noradId")) {
        sendError(request, "Missing noradId parameter", 400);
        return;
    }
    
    uint32_t noradId = request->getParam("noradId")->value().toInt();
    
    SatNOGSTransmitter transmitters[20];
    int count = externalAPI.getTransmitters(noradId, transmitters, 20);
    
    StaticJsonDocument<JSON_SIZE_XLARGE> doc;
    JsonArray txArray = doc["transmitters"].to<JsonArray>();
    
    for (int i = 0; i < count; i++) {
        JsonObject t = txArray.createNestedObject();
        t["description"] = transmitters[i].description;
        t["downlinkLow"] = transmitters[i].downlinkLow;
        t["downlinkHigh"] = transmitters[i].downlinkHigh;
        t["uplinkLow"] = transmitters[i].uplinkLow;
        t["uplinkHigh"] = transmitters[i].uplinkHigh;
        t["mode"] = transmitters[i].mode;
        t["alive"] = transmitters[i].alive;
        t["baud"] = transmitters[i].baud;
    }
    
    doc["count"] = count;
    
    sendJSON(request, doc);
}

void WebServer::handleFetchTLEGroup(AsyncWebServerRequest* request, JsonVariant& json) {
    const char* group = json["group"];
    if (!group) {
        sendError(request, "Missing group parameter", 400);
        return;
    }
    
    int count = externalAPI.fetchTLEGroup(group, nullptr);
    
    StaticJsonDocument<JSON_SIZE_SMALL> doc;
    doc["fetched"] = count;
    doc["group"] = group;
    sendJSON(request, doc);
}

// =============================================================================
// EME MODE HANDLERS
// =============================================================================

void WebServer::handleEnableEME(AsyncWebServerRequest* request) {
    if (trackingEngine.enableEMEMode()) {
        sendOK(request, "EME mode enabled");
    } else {
        sendError(request, "Cannot enable EME - moon below horizon", 400);
    }
}

void WebServer::handleDisableEME(AsyncWebServerRequest* request) {
    trackingEngine.disableEMEMode();
    sendOK(request, "EME mode disabled");
}

void WebServer::handleGetMoonInfo(AsyncWebServerRequest* request) {
    float moonAz, moonEl;
    trackingEngine.getMoonPosition(moonAz, moonEl);
    
    StaticJsonDocument<JSON_SIZE_SMALL> doc;
    doc["azimuth"] = moonAz;
    doc["elevation"] = moonEl;
    doc["visible"] = moonEl > 0;
    doc["distance"] = trackingEngine.getMoonDistance();
    doc["emeMode"] = trackingEngine.isEMEMode();
    
    sendJSON(request, doc);
}

void WebServer::handleGetEMEPathLoss(AsyncWebServerRequest* request) {
    float freq = request->hasParam("freq") ?
                  request->getParam("freq")->value().toFloat() : 144.0;
    
    StaticJsonDocument<JSON_SIZE_SMALL> doc;
    doc["frequency"] = freq;
    doc["pathLoss"] = trackingEngine.getEMEPathLoss(freq);
    doc["distance"] = trackingEngine.getMoonDistance();
    
    sendJSON(request, doc);
}

// =============================================================================
// GEO SATELLITE HANDLERS
// =============================================================================

void WebServer::handlePointToGeo(AsyncWebServerRequest* request, JsonVariant& json) {
    float longitude = json["longitude"] | 0;
    const char* name = json["name"] | "Geo Satellite";
    
    if (trackingEngine.pointToGeo(longitude, name)) {
        sendOK(request, "Pointing to geostationary position");
    } else {
        sendError(request, "Geostationary position not visible", 400);
    }
}

void WebServer::handleEnableGeoTracking(AsyncWebServerRequest* request, JsonVariant& json) {
    GeoSatConfig config;
    config.longitude = json["longitude"] | 0;
    config.noradId = json["noradId"] | 0;
    strncpy(config.name, json["name"] | "Geo Sat", sizeof(config.name) - 1);
    
    if (trackingEngine.enableGeoTracking(config)) {
        sendOK(request, "Geo tracking enabled");
    } else {
        sendError(request, "Cannot enable geo tracking - position not visible", 400);
    }
}

void WebServer::handleDisableGeoTracking(AsyncWebServerRequest* request) {
    trackingEngine.disableGeoTracking();
    sendOK(request, "Geo tracking disabled");
}

// =============================================================================
// ANTENNA PATTERN HANDLERS
// =============================================================================

void WebServer::handleGetAntennaPattern(AsyncWebServerRequest* request) {
    AntennaPattern pattern = trackingEngine.getAntennaPattern();
    
    StaticJsonDocument<JSON_SIZE_SMALL> doc;
    doc["azOffset"] = pattern.azBeamOffset;
    doc["elOffset"] = pattern.elBeamOffset;
    doc["beamwidth"] = pattern.beamwidth;
    doc["enabled"] = pattern.enabled;
    
    sendJSON(request, doc);
}

void WebServer::handleSetAntennaPattern(AsyncWebServerRequest* request, JsonVariant& json) {
    AntennaPattern pattern = trackingEngine.getAntennaPattern();
    
    if (json.containsKey("azOffset")) {
        pattern.azBeamOffset = json["azOffset"].as<float>();
    }
    if (json.containsKey("elOffset")) {
        pattern.elBeamOffset = json["elOffset"].as<float>();
    }
    if (json.containsKey("beamwidth")) {
        pattern.beamwidth = json["beamwidth"].as<float>();
    }
    if (json.containsKey("enabled")) {
        pattern.enabled = json["enabled"].as<bool>();
    }
    
    trackingEngine.setAntennaPattern(pattern);
    sendOK(request, "Antenna pattern updated");
}

// =============================================================================
// PRE-POSITIONING HANDLER
// =============================================================================

void WebServer::handleSetPrePosition(AsyncWebServerRequest* request, JsonVariant& json) {
    bool enabled = json["enabled"] | true;
    uint32_t seconds = json["seconds"] | PRE_POSITION_SECONDS;
    
    trackingEngine.setPrePositioning(enabled, seconds);
    sendOK(request, "Pre-positioning settings updated");
}
