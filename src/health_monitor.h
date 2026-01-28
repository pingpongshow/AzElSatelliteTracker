/**
 * @file health_monitor.h
 * @brief System health monitoring and watchdog management
 * 
 * Monitors GPS, motors, network, and memory health with
 * automatic recovery attempts and watchdog protection.
 */

#ifndef HEALTH_MONITOR_H
#define HEALTH_MONITOR_H

#include <Arduino.h>
#include <esp_task_wdt.h>
#include "config.h"

// Health issue types
enum class HealthIssue {
    NONE,
    GPS_STALE,
    GPS_NO_FIX,
    MOTOR_STALL,
    MOTOR_OVERTEMP,
    MOTOR_COMM_FAIL,
    WIFI_WEAK_SIGNAL,
    WIFI_DISCONNECTED,
    HEAP_LOW,
    HEAP_FRAGMENTED,
    PSRAM_LOW,
    NVS_ERROR,
    CAMERA_FAIL
};

// Health status structure
struct HealthStatus {
    bool gpsHealthy;
    bool motorAzHealthy;
    bool motorElHealthy;
    bool wifiHealthy;
    bool memoryHealthy;
    bool cameraHealthy;
    bool overallHealthy;
    
    HealthIssue lastIssue;
    uint32_t issueCount;
    uint32_t lastIssueTime;
    
    // Detailed metrics
    int32_t wifiRSSI;
    uint32_t freeHeap;
    uint32_t minFreeHeap;
    uint32_t freePsram;
    uint32_t gpsAge;
    int16_t motorAzTemp;
    int16_t motorElTemp;
    uint32_t uptime;
};

// Health event log entry
struct HealthEvent {
    uint32_t timestamp;
    HealthIssue issue;
    bool resolved;
    char description[64];
};

/**
 * @class HealthMonitor
 * @brief System health monitoring and watchdog management
 */
class HealthMonitor {
public:
    HealthMonitor();
    
    /**
     * @brief Initialize health monitor and watchdog
     * @param enableWatchdog Enable hardware watchdog
     * @return true if initialization successful
     */
    bool begin(bool enableWatchdog = true);
    
    /**
     * @brief Periodic health check update
     * Call from main loop
     */
    void update();
    
    /**
     * @brief Feed the watchdog timer
     * Call regularly to prevent reset
     */
    void feedWatchdog();
    
    /**
     * @brief Get current health status
     */
    HealthStatus getStatus() const { return _status; }
    
    /**
     * @brief Check if system is healthy
     */
    bool isHealthy() const { return _status.overallHealthy; }
    
    /**
     * @brief Get last health issue
     */
    HealthIssue getLastIssue() const { return _status.lastIssue; }
    
    /**
     * @brief Get health event log
     * @param events Output array
     * @param maxEvents Maximum events to return
     * @return Number of events returned
     */
    int getEventLog(HealthEvent* events, int maxEvents);
    
    /**
     * @brief Clear event log
     */
    void clearEventLog();
    
    /**
     * @brief Enable/disable auto-recovery
     */
    void setAutoRecovery(bool enable) { _autoRecovery = enable; }
    
    /**
     * @brief Force a health check now
     */
    void checkNow();
    
    /**
     * @brief Get uptime in seconds
     */
    uint32_t getUptime() const { return millis() / 1000; }
    
    /**
     * @brief Get formatted status string
     */
    String getStatusString() const;

private:
    HealthStatus _status;
    bool _watchdogEnabled;
    bool _autoRecovery;
    uint32_t _lastCheckMs;
    
    // Event log (circular buffer)
    static const int MAX_EVENTS = 50;
    HealthEvent _eventLog[MAX_EVENTS];
    int _eventHead;
    int _eventCount;
    
    // Index of last unresolved event per issue type for O(1) recovery lookup
    // Using -1 to indicate no unresolved event of that type
    static const int NUM_ISSUE_TYPES = 13;  // Number of HealthIssue enum values
    int _lastUnresolvedIndex[NUM_ISSUE_TYPES];
    
    // Motor UART tracking - skip frequent checks when UART is broken
    // This prevents motion pauses caused by UART timeout every 5 seconds
    bool _motorUartWorking;
    uint8_t _motorUartFailCount;
    
    // Health check methods
    bool checkGPSHealth();
    bool checkMotorHealth();
    bool checkNetworkHealth();
    bool checkMemoryHealth();
    bool checkCameraHealth();
    
    // Recovery methods
    void attemptRecovery(HealthIssue issue);
    void recoverGPS();
    void recoverMotor();
    void recoverNetwork();
    void recoverCamera();
    
    // Logging
    void logEvent(HealthIssue issue, const char* description);
    void logRecovery(HealthIssue issue);
};

// Global instance
extern HealthMonitor healthMonitor;

#endif // HEALTH_MONITOR_H
