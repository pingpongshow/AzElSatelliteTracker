/**
 * @file web_handlers_system.cpp
 * @brief Web server handlers for system functionality
 * 
 * This file contains WebServer method implementations for:
 * - Health monitoring (status, events)
 * - SD card operations (TLE import/export, config backup/restore, pass logs)
 * - UDP broadcast configuration
 * 
 * Separated from web_server.cpp for better maintainability.
 */

#include "web_server.h"
#include "health_monitor.h"
#include "sd_storage.h"
#include "udp_broadcast.h"
#include "rotctld_server.h"

// =============================================================================
// HEALTH MONITORING HANDLERS
// =============================================================================

void WebServer::handleGetHealth(AsyncWebServerRequest* request) {
    HealthStatus health = healthMonitor.getStatus();
    
    StaticJsonDocument<JSON_SIZE_LARGE> doc;
    doc["status"] = health.overallHealthy ? 0 : 1;
    doc["gpsValid"] = health.gpsHealthy;
    doc["gpsAge"] = health.gpsAge;
    doc["gpsSats"] = 0;  // Not tracked in current implementation
    doc["motorsEnabled"] = true;
    doc["azMotorOk"] = health.motorAzHealthy;
    doc["elMotorOk"] = health.motorElHealthy;
    doc["wifiConnected"] = health.wifiHealthy;
    doc["wifiRSSI"] = health.wifiRSSI;
    doc["freeHeap"] = health.freeHeap;
    doc["minFreeHeap"] = health.minFreeHeap;
    doc["freePsram"] = health.freePsram;
    doc["heapFrag"] = 0;  // Calculated separately if needed
    doc["sdMounted"] = sdStorage.isAvailable();
    doc["uptime"] = health.uptime;
    doc["wsClients"] = getClientCount();
    doc["rotctldClients"] = rotctldServer.getClientCount();
    
    sendJSON(request, doc);
}

void WebServer::handleGetHealthEvents(AsyncWebServerRequest* request) {
    HealthEvent events[32];
    int count = healthMonitor.getEventLog(events, 32);
    
    StaticJsonDocument<JSON_SIZE_XXLARGE> doc;
    JsonArray eventArray = doc["events"].to<JsonArray>();
    
    for (int i = 0; i < count; i++) {
        JsonObject e = eventArray.createNestedObject();
        e["time"] = events[i].timestamp;
        e["issue"] = (int)events[i].issue;
        e["resolved"] = events[i].resolved;
        e["message"] = events[i].description;
    }
    
    doc["count"] = count;
    
    sendJSON(request, doc);
}

void WebServer::handleClearHealthEvents(AsyncWebServerRequest* request) {
    healthMonitor.clearEventLog();
    sendOK(request, "Health events cleared");
}

// =============================================================================
// SD CARD HANDLERS
// =============================================================================

void WebServer::handleSDStatus(AsyncWebServerRequest* request) {
    StaticJsonDocument<JSON_SIZE_SMALL> doc;
    doc["available"] = sdStorage.isAvailable();
    
    if (sdStorage.isAvailable()) {
        uint32_t totalMB, usedMB;
        sdStorage.getCardInfo(totalMB, usedMB);
        doc["totalMB"] = totalMB;
        doc["usedMB"] = usedMB;
        doc["freeMB"] = totalMB - usedMB;
        doc["hasBackup"] = sdStorage.hasConfigBackup();
    }
    
    sendJSON(request, doc);
}

void WebServer::handleSDListTLEs(AsyncWebServerRequest* request) {
    if (!sdStorage.isAvailable()) {
        sendError(request, "SD card not available", 503);
        return;
    }
    
    String names[50];
    int count = sdStorage.listTLEs(names, 50);
    
    StaticJsonDocument<JSON_SIZE_XLARGE> doc;
    JsonArray tleArray = doc["tles"].to<JsonArray>();
    
    for (int i = 0; i < count; i++) {
        tleArray.add(names[i]);
    }
    
    doc["count"] = count;
    
    sendJSON(request, doc);
}

void WebServer::handleSDImportTLE(AsyncWebServerRequest* request, JsonVariant& json) {
    if (!sdStorage.isAvailable()) {
        sendError(request, "SD card not available", 503);
        return;
    }
    
    const char* filename = json["filename"] | "/tle_import.txt";
    int count = sdStorage.importTLEFile(filename);
    
    StaticJsonDocument<JSON_SIZE_TINY> doc;
    doc["imported"] = count;
    sendJSON(request, doc);
}

void WebServer::handleSDExportTLEs(AsyncWebServerRequest* request) {
    if (!sdStorage.isAvailable()) {
        sendError(request, "SD card not available", 503);
        return;
    }
    
    int count = sdStorage.exportTLEFile("/tle_export.txt");
    
    StaticJsonDocument<JSON_SIZE_SMALL> doc;
    doc["exported"] = count;
    doc["filename"] = "/tle_export.txt";
    sendJSON(request, doc);
}

void WebServer::handleSDBackupConfig(AsyncWebServerRequest* request) {
    if (!sdStorage.isAvailable()) {
        sendError(request, "SD card not available", 503);
        return;
    }
    
    if (sdStorage.backupConfig()) {
        sendOK(request, "Configuration backed up to SD");
    } else {
        sendError(request, "Backup failed", 500);
    }
}

void WebServer::handleSDRestoreConfig(AsyncWebServerRequest* request) {
    if (!sdStorage.isAvailable()) {
        sendError(request, "SD card not available", 503);
        return;
    }
    
    if (sdStorage.restoreConfig()) {
        sendOK(request, "Configuration restored from SD");
    } else {
        sendError(request, "Restore failed - no backup found", 404);
    }
}

void WebServer::handleGetPassLog(AsyncWebServerRequest* request) {
    if (!sdStorage.isAvailable()) {
        sendError(request, "SD card not available", 503);
        return;
    }
    
    PassLogEntry entries[20];
    int count = sdStorage.getPassHistory(entries, 20);
    
    StaticJsonDocument<JSON_SIZE_XXLARGE> doc;
    JsonArray logArray = doc["passes"].to<JsonArray>();
    
    for (int i = 0; i < count; i++) {
        JsonObject e = logArray.createNestedObject();
        e["timestamp"] = entries[i].timestamp;
        e["satellite"] = entries[i].satellite;
        e["duration"] = entries[i].duration;
        e["maxEl"] = entries[i].maxElevation;
        e["success"] = entries[i].trackingSuccess;
        e["signal"] = entries[i].signalQuality;
    }
    
    doc["count"] = count;
    
    // Get stats
    int totalPasses, successfulPasses;
    float avgMaxEl;
    sdStorage.getPassStats(totalPasses, successfulPasses, avgMaxEl);
    doc["totalPasses"] = totalPasses;
    doc["successfulPasses"] = successfulPasses;
    doc["avgMaxEl"] = avgMaxEl;
    
    sendJSON(request, doc);
}

void WebServer::handleExportPassLog(AsyncWebServerRequest* request) {
    if (!sdStorage.isAvailable()) {
        sendError(request, "SD card not available", 503);
        return;
    }
    
    if (sdStorage.exportPassLogCSV("/passlog.csv")) {
        sendOK(request, "Pass log exported to /passlog.csv");
    } else {
        sendError(request, "Export failed", 500);
    }
}

// =============================================================================
// UDP BROADCAST HANDLERS
// =============================================================================

void WebServer::handleUDPConfig(AsyncWebServerRequest* request, JsonVariant& json) {
    if (json.containsKey("enabled")) {
        udpBroadcast.setEnabled(json["enabled"].as<bool>());
    }
    if (json.containsKey("interval")) {
        udpBroadcast.setInterval(json["interval"].as<uint32_t>());
    }
    if (json.containsKey("format")) {
        udpBroadcast.setFormat((UDPFormat)json["format"].as<int>());
    }
    
    sendOK(request, "UDP config updated");
}

void WebServer::handleUDPStatus(AsyncWebServerRequest* request) {
    StaticJsonDocument<JSON_SIZE_SMALL> doc;
    doc["enabled"] = udpBroadcast.isEnabled();
    doc["port"] = udpBroadcast.getPort();
    doc["broadcasts"] = udpBroadcast.getBroadcastCount();
    
    sendJSON(request, doc);
}
