/**
 * @file sd_storage.cpp
 * @brief SD card storage implementation
 */

#include "sd_storage.h"
#include "nvs_storage.h"
#include <ArduinoJson.h>

// Global instance
SDStorage sdStorage;

SDStorage::SDStorage()
    : _available(false)
    , _csPin(SD_CS_PIN)
{
}

bool SDStorage::begin(int8_t csPin) {
    DEBUG_PRINTLN("SDStorage: Initializing...");
    
    _csPin = csPin;
    
    if (_csPin < 0) {
        DEBUG_PRINTLN("SDStorage: No CS pin configured, SD disabled");
        return false;
    }
    
    // Initialize SPI and SD
    if (!SD.begin(_csPin, SPI, SD_SPI_FREQ_MHZ * 1000000)) {
        DEBUG_PRINTLN("SDStorage: SD card initialization failed");
        _available = false;
        return false;
    }
    
    _available = true;
    
    // Ensure directories exist
    ensureDirectory(TLE_STORAGE_DIR);
    ensureDirectory(PASS_LOG_DIR);
    
    uint8_t cardType = SD.cardType();
    DEBUG_PRINTF("SDStorage: Card type: %d\n", cardType);
    
    uint32_t totalMB, usedMB;
    getCardInfo(totalMB, usedMB);
    DEBUG_PRINTF("SDStorage: %lu MB total, %lu MB used\n", totalMB, usedMB);
    
    DEBUG_PRINTLN("SDStorage: Initialization complete");
    return true;
}

void SDStorage::getCardInfo(uint32_t& totalMB, uint32_t& usedMB) {
    if (!_available) {
        totalMB = 0;
        usedMB = 0;
        return;
    }
    
    totalMB = SD.totalBytes() / (1024 * 1024);
    usedMB = SD.usedBytes() / (1024 * 1024);
}

bool SDStorage::ensureDirectory(const char* path) {
    if (!_available) return false;
    
    if (!SD.exists(path)) {
        return SD.mkdir(path);
    }
    return true;
}

String SDStorage::sanitizeFilename(const char* name) {
    String safe = "";
    for (int i = 0; name[i] && i < 20; i++) {
        char c = name[i];
        if (isalnum(c) || c == '-' || c == '_') {
            safe += c;
        } else if (c == ' ') {
            safe += '_';
        }
    }
    return safe;
}

String SDStorage::getTLEFilename(const char* name) {
    return String(TLE_STORAGE_DIR) + "/" + sanitizeFilename(name) + ".tle";
}

// TLE Storage Implementation

bool SDStorage::saveTLE(const TLEEntry& entry) {
    if (!_available) return false;
    
    String filename = getTLEFilename(entry.name);
    
    File file = SD.open(filename, FILE_WRITE);
    if (!file) {
        DEBUG_PRINTF("SDStorage: Failed to create TLE file: %s\n", filename.c_str());
        return false;
    }
    
    writeTLEFile(file, entry);
    file.close();
    
    DEBUG_PRINTF("SDStorage: Saved TLE: %s\n", entry.name);
    return true;
}

void SDStorage::writeTLEFile(File& file, const TLEEntry& entry) {
    file.println(entry.name);
    file.println(entry.line1);
    file.println(entry.line2);
    file.printf("# Fetched: %lu\n", entry.fetchTime);
}

bool SDStorage::loadTLE(const char* name, TLEEntry& entry) {
    if (!_available) return false;
    
    String filename = getTLEFilename(name);
    
    File file = SD.open(filename, FILE_READ);
    if (!file) {
        return false;
    }
    
    bool success = parseTLEFile(file, entry);
    file.close();
    
    return success;
}

bool SDStorage::parseTLEFile(File& file, TLEEntry& entry) {
    memset(&entry, 0, sizeof(TLEEntry));
    
    // Read name (line 0)
    if (!file.available()) return false;
    String name = file.readStringUntil('\n');
    name.trim();
    strncpy(entry.name, name.c_str(), sizeof(entry.name) - 1);
    
    // Read line 1
    if (!file.available()) return false;
    String line1 = file.readStringUntil('\n');
    line1.trim();
    if (line1.length() < 69 || line1[0] != '1') return false;
    strncpy(entry.line1, line1.c_str(), sizeof(entry.line1) - 1);
    
    // Read line 2
    if (!file.available()) return false;
    String line2 = file.readStringUntil('\n');
    line2.trim();
    if (line2.length() < 69 || line2[0] != '2') return false;
    strncpy(entry.line2, line2.c_str(), sizeof(entry.line2) - 1);
    
    // Try to read fetch time from comment
    while (file.available()) {
        String line = file.readStringUntil('\n');
        if (line.startsWith("# Fetched:")) {
            entry.fetchTime = line.substring(10).toInt();
            break;
        }
    }
    
    entry.valid = true;
    return true;
}

bool SDStorage::deleteTLE(const char* name) {
    if (!_available) return false;
    
    String filename = getTLEFilename(name);
    return SD.remove(filename);
}

int SDStorage::listTLEs(String* names, int maxCount) {
    if (!_available) return 0;
    
    File dir = SD.open(TLE_STORAGE_DIR);
    if (!dir || !dir.isDirectory()) {
        return 0;
    }
    
    int count = 0;
    File file;
    
    while ((file = dir.openNextFile()) && count < maxCount) {
        String filename = file.name();
        if (filename.endsWith(".tle")) {
            // Extract name from filename
            TLEEntry entry;
            if (parseTLEFile(file, entry)) {
                names[count++] = String(entry.name);
            }
        }
        file.close();
    }
    
    dir.close();
    return count;
}

int SDStorage::importTLEFile(const char* filename, void (*callback)(const TLEEntry&)) {
    if (!_available) return 0;
    
    File file = SD.open(filename, FILE_READ);
    if (!file) {
        DEBUG_PRINTF("SDStorage: Cannot open import file: %s\n", filename);
        return 0;
    }
    
    int count = 0;
    
    while (file.available()) {
        TLEEntry entry;
        
        // Read name
        String name = file.readStringUntil('\n');
        name.trim();
        if (name.length() == 0 || name[0] == '#') continue;
        
        // Skip if looks like TLE line
        if (name[0] == '1' || name[0] == '2') continue;
        
        strncpy(entry.name, name.c_str(), sizeof(entry.name) - 1);
        
        // Read line 1
        if (!file.available()) break;
        String line1 = file.readStringUntil('\n');
        line1.trim();
        if (line1.length() < 69 || line1[0] != '1') continue;
        strncpy(entry.line1, line1.c_str(), sizeof(entry.line1) - 1);
        
        // Read line 2
        if (!file.available()) break;
        String line2 = file.readStringUntil('\n');
        line2.trim();
        if (line2.length() < 69 || line2[0] != '2') continue;
        strncpy(entry.line2, line2.c_str(), sizeof(entry.line2) - 1);
        
        entry.fetchTime = millis() / 1000;
        entry.valid = true;
        
        // Save to SD
        saveTLE(entry);
        
        // Callback
        if (callback) {
            callback(entry);
        }
        
        count++;
    }
    
    file.close();
    DEBUG_PRINTF("SDStorage: Imported %d TLEs from %s\n", count, filename);
    return count;
}

int SDStorage::exportTLEFile(const char* filename) {
    if (!_available) return 0;
    
    File outFile = SD.open(filename, FILE_WRITE);
    if (!outFile) {
        return 0;
    }
    
    File dir = SD.open(TLE_STORAGE_DIR);
    if (!dir) {
        outFile.close();
        return 0;
    }
    
    int count = 0;
    File file;
    
    while ((file = dir.openNextFile())) {
        String fname = file.name();
        if (fname.endsWith(".tle")) {
            TLEEntry entry;
            if (parseTLEFile(file, entry)) {
                writeTLEFile(outFile, entry);
                outFile.println();  // Blank line between entries
                count++;
            }
        }
        file.close();
    }
    
    dir.close();
    outFile.close();
    
    DEBUG_PRINTF("SDStorage: Exported %d TLEs to %s\n", count, filename);
    return count;
}

// Pass Logging Implementation

bool SDStorage::logPass(const PassLogEntry& entry) {
    if (!_available) return false;
    
    // Create filename based on date
    char filename[32];
    snprintf(filename, sizeof(filename), "%s/passes.log", PASS_LOG_DIR);
    
    File file = SD.open(filename, FILE_APPEND);
    if (!file) {
        return false;
    }
    
    // Write as JSON line
    StaticJsonDocument<2048> doc;
    doc["ts"] = entry.timestamp;
    doc["sat"] = entry.satellite;
    doc["dur"] = entry.duration;
    doc["maxEl"] = entry.maxElevation;
    doc["aosAz"] = entry.aosAzimuth;
    doc["losAz"] = entry.losAzimuth;
    doc["ok"] = entry.trackingSuccess;
    doc["sig"] = entry.signalQuality;
    if (strlen(entry.notes) > 0) {
        doc["notes"] = entry.notes;
    }
    
    serializeJson(doc, file);
    file.println();
    file.close();
    
    return true;
}

int SDStorage::getPassHistory(PassLogEntry* entries, int maxEntries, uint32_t sinceTimestamp) {
    if (!_available) return 0;
    
    char filename[32];
    snprintf(filename, sizeof(filename), "%s/passes.log", PASS_LOG_DIR);
    
    File file = SD.open(filename, FILE_READ);
    if (!file) {
        return 0;
    }
    
    int count = 0;
    
    while (file.available() && count < maxEntries) {
        String line = file.readStringUntil('\n');
        if (line.length() == 0) continue;
        
        StaticJsonDocument<2048> doc;
        DeserializationError error = deserializeJson(doc, line);
        if (error) continue;
        
        uint32_t ts = doc["ts"] | 0;
        if (sinceTimestamp > 0 && ts < sinceTimestamp) continue;
        
        entries[count].timestamp = ts;
        strncpy(entries[count].satellite, doc["sat"] | "", sizeof(entries[count].satellite) - 1);
        entries[count].duration = doc["dur"] | 0;
        entries[count].maxElevation = doc["maxEl"] | 0;
        entries[count].aosAzimuth = doc["aosAz"] | 0;
        entries[count].losAzimuth = doc["losAz"] | 0;
        entries[count].trackingSuccess = doc["ok"] | false;
        entries[count].signalQuality = doc["sig"] | -1;
        strncpy(entries[count].notes, doc["notes"] | "", sizeof(entries[count].notes) - 1);
        
        count++;
    }
    
    file.close();
    return count;
}

void SDStorage::getPassStats(int& totalPasses, int& successfulPasses, float& avgMaxEl) {
    totalPasses = 0;
    successfulPasses = 0;
    avgMaxEl = 0;
    
    if (!_available) return;
    
    char filename[32];
    snprintf(filename, sizeof(filename), "%s/passes.log", PASS_LOG_DIR);
    
    File file = SD.open(filename, FILE_READ);
    if (!file) return;
    
    float totalEl = 0;
    
    while (file.available()) {
        String line = file.readStringUntil('\n');
        if (line.length() == 0) continue;
        
        StaticJsonDocument<2048> doc;
        if (deserializeJson(doc, line)) continue;
        
        totalPasses++;
        if (doc["ok"] | false) successfulPasses++;
        totalEl += doc["maxEl"] | 0;
    }
    
    file.close();
    
    if (totalPasses > 0) {
        avgMaxEl = totalEl / totalPasses;
    }
}

bool SDStorage::exportPassLogCSV(const char* filename) {
    if (!_available) return false;
    
    File outFile = SD.open(filename, FILE_WRITE);
    if (!outFile) return false;
    
    // Write header
    outFile.println("Timestamp,Satellite,Duration,MaxElevation,AOSAzimuth,LOSAzimuth,Success,SignalQuality,Notes");
    
    char logFile[32];
    snprintf(logFile, sizeof(logFile), "%s/passes.log", PASS_LOG_DIR);
    
    File inFile = SD.open(logFile, FILE_READ);
    if (!inFile) {
        outFile.close();
        return false;
    }
    
    while (inFile.available()) {
        String line = inFile.readStringUntil('\n');
        if (line.length() == 0) continue;
        
        StaticJsonDocument<2048> doc;
        if (deserializeJson(doc, line)) continue;
        
        outFile.printf("%lu,%s,%lu,%.1f,%.1f,%.1f,%s,%d,%s\n",
            (uint32_t)(doc["ts"] | 0),
            (const char*)(doc["sat"] | ""),
            (uint32_t)(doc["dur"] | 0),
            (float)(doc["maxEl"] | 0),
            (float)(doc["aosAz"] | 0),
            (float)(doc["losAz"] | 0),
            (doc["ok"] | false) ? "Yes" : "No",
            (int)(doc["sig"] | -1),
            (const char*)(doc["notes"] | ""));
    }
    
    inFile.close();
    outFile.close();
    return true;
}

void SDStorage::clearPassLog() {
    if (!_available) return;
    
    char filename[32];
    snprintf(filename, sizeof(filename), "%s/passes.log", PASS_LOG_DIR);
    SD.remove(filename);
}

// Configuration Backup

bool SDStorage::backupConfig() {
    if (!_available) return false;
    
    TrackerConfig config;
    if (!nvsStorage.loadConfig(config)) {
        return false;
    }
    
    File file = SD.open(CONFIG_BACKUP_FILE, FILE_WRITE);
    if (!file) return false;
    
    StaticJsonDocument<2048> doc;
    doc["azOffset"] = config.azimuthOffset;
    doc["elOffset"] = config.elevationOffset;
    doc["lat"] = config.latitude;
    doc["lon"] = config.longitude;
    doc["alt"] = config.altitude;
    doc["manualLoc"] = config.useManualLocation;
    doc["runCurrent"] = config.motorRunCurrent;
    doc["holdCurrent"] = config.motorHoldCurrent;
    doc["microsteps"] = config.microsteps;
    doc["azMin"] = config.azMin;
    doc["azMax"] = config.azMax;
    doc["elMin"] = config.elMin;
    doc["elMax"] = config.elMax;
    doc["parkAz"] = config.parkAz;
    doc["parkEl"] = config.parkEl;
    
    serializeJsonPretty(doc, file);
    file.close();
    
    DEBUG_PRINTLN("SDStorage: Configuration backed up");
    return true;
}

bool SDStorage::restoreConfig() {
    if (!_available) return false;
    
    File file = SD.open(CONFIG_BACKUP_FILE, FILE_READ);
    if (!file) return false;
    
    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error) {
        DEBUG_PRINTF("SDStorage: Config parse error: %s\n", error.c_str());
        return false;
    }
    
    TrackerConfig config;
    config.azimuthOffset = doc["azOffset"] | 0.0f;
    config.elevationOffset = doc["elOffset"] | 0.0f;
    config.latitude = doc["lat"] | DEFAULT_LATITUDE;
    config.longitude = doc["lon"] | DEFAULT_LONGITUDE;
    config.altitude = doc["alt"] | DEFAULT_ALTITUDE;
    config.useManualLocation = doc["manualLoc"] | false;
    config.motorRunCurrent = doc["runCurrent"] | MOTOR_RUN_CURRENT_MA;
    config.motorHoldCurrent = doc["holdCurrent"] | MOTOR_HOLD_CURRENT_MA;
    config.microsteps = doc["microsteps"] | TMC_MICROSTEPS;
    config.azMin = doc["azMin"] | AZ_MIN_DEG;
    config.azMax = doc["azMax"] | AZ_MAX_DEG;
    config.elMin = doc["elMin"] | EL_MIN_DEG;
    config.elMax = doc["elMax"] | EL_MAX_DEG;
    config.parkAz = doc["parkAz"] | PARK_AZ_DEG;
    config.parkEl = doc["parkEl"] | PARK_EL_DEG;
    config.positionValid = false;
    config.lastSatellite[0] = '\0';
    
    nvsStorage.saveConfig(config);
    
    DEBUG_PRINTLN("SDStorage: Configuration restored");
    return true;
}

bool SDStorage::hasConfigBackup() {
    if (!_available) return false;
    return SD.exists(CONFIG_BACKUP_FILE);
}
