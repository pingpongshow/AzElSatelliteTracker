/**
 * @file sd_storage.h
 * @brief SD card storage for TLEs, logs, and configuration
 * 
 * Provides persistent storage on SD card for:
 * - TLE data files
 * - Pass logging
 * - Configuration backup
 */

#ifndef SD_STORAGE_H
#define SD_STORAGE_H

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "config.h"
#include "tle_manager.h"

// Pass log entry structure
struct PassLogEntry {
    uint32_t timestamp;          // Unix time of AOS
    char satellite[25];          // Satellite name
    uint32_t duration;           // Pass duration in seconds
    float maxElevation;          // Maximum elevation achieved
    float aosAzimuth;            // AOS azimuth
    float losAzimuth;            // LOS azimuth
    bool trackingSuccess;        // Was tracking successful
    int8_t signalQuality;        // -1 = unknown, 0-100 = quality
    char notes[64];              // User notes
};

/**
 * @class SDStorage
 * @brief SD card storage management
 */
class SDStorage {
public:
    SDStorage();
    
    /**
     * @brief Initialize SD card
     * @param csPin Chip select pin (-1 for default)
     * @return true if SD card initialized
     */
    bool begin(int8_t csPin = SD_CS_PIN);
    
    /**
     * @brief Check if SD card is available
     */
    bool isAvailable() const { return _available; }
    
    /**
     * @brief Get SD card info
     * @param totalMB Total size in MB
     * @param usedMB Used space in MB
     */
    void getCardInfo(uint32_t& totalMB, uint32_t& usedMB);
    
    // TLE Storage
    /**
     * @brief Save TLE to SD card
     * @param entry TLE entry to save
     * @return true if successful
     */
    bool saveTLE(const TLEEntry& entry);
    
    /**
     * @brief Load TLE from SD card
     * @param name Satellite name
     * @param entry Output TLE entry
     * @return true if found and loaded
     */
    bool loadTLE(const char* name, TLEEntry& entry);
    
    /**
     * @brief Delete TLE from SD card
     * @param name Satellite name
     * @return true if deleted
     */
    bool deleteTLE(const char* name);
    
    /**
     * @brief List all TLEs on SD card
     * @param names Output array of names
     * @param maxCount Maximum names to return
     * @return Number of TLEs found
     */
    int listTLEs(String* names, int maxCount);
    
    /**
     * @brief Import TLEs from file (3LE format)
     * @param filename File to import from
     * @param callback Called for each TLE imported
     * @return Number of TLEs imported
     */
    int importTLEFile(const char* filename, void (*callback)(const TLEEntry&) = nullptr);
    
    /**
     * @brief Export all TLEs to file (3LE format)
     * @param filename File to export to
     * @return Number of TLEs exported
     */
    int exportTLEFile(const char* filename);
    
    // Pass Logging
    /**
     * @brief Log a satellite pass
     * @param entry Pass log entry
     * @return true if logged successfully
     */
    bool logPass(const PassLogEntry& entry);
    
    /**
     * @brief Get pass history
     * @param entries Output array
     * @param maxEntries Maximum entries to return
     * @param sinceTimestamp Only return passes after this time (0 = all)
     * @return Number of entries returned
     */
    int getPassHistory(PassLogEntry* entries, int maxEntries, uint32_t sinceTimestamp = 0);
    
    /**
     * @brief Get pass statistics
     * @param totalPasses Total passes logged
     * @param successfulPasses Successful tracking passes
     * @param avgMaxEl Average maximum elevation
     */
    void getPassStats(int& totalPasses, int& successfulPasses, float& avgMaxEl);
    
    /**
     * @brief Export pass log to CSV
     * @param filename Output filename
     * @return true if exported successfully
     */
    bool exportPassLogCSV(const char* filename);
    
    /**
     * @brief Clear pass log
     */
    void clearPassLog();
    
    // Configuration Backup
    /**
     * @brief Backup configuration to SD
     * @return true if backed up successfully
     */
    bool backupConfig();
    
    /**
     * @brief Restore configuration from SD
     * @return true if restored successfully
     */
    bool restoreConfig();
    
    /**
     * @brief Check if config backup exists
     */
    bool hasConfigBackup();

private:
    bool _available;
    int8_t _csPin;
    
    // Helper methods
    String sanitizeFilename(const char* name);
    String getTLEFilename(const char* name);
    bool ensureDirectory(const char* path);
    bool parseTLEFile(File& file, TLEEntry& entry);
    void writeTLEFile(File& file, const TLEEntry& entry);
};

// Global instance
extern SDStorage sdStorage;

#endif // SD_STORAGE_H
