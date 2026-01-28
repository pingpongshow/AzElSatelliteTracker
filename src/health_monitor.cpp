/**
 * @file health_monitor.cpp
 * @brief System health monitoring implementation
 */

#include "health_monitor.h"
#include "gps.h"
#include "stepper_control.h"
#include "wifi_manager.h"
#include <WiFi.h>

// Global instance
HealthMonitor healthMonitor;

HealthMonitor::HealthMonitor()
    : _watchdogEnabled(false)
    , _autoRecovery(false)
    , _lastCheckMs(0)
    , _eventHead(0)
    , _eventCount(0)
    , _motorUartWorking(true)
    , _motorUartFailCount(0)
{
    memset(&_status, 0, sizeof(HealthStatus));
    _status.overallHealthy = true;
    
    // Initialize issue index array to -1 (no unresolved events)
    for (int i = 0; i < NUM_ISSUE_TYPES; i++) {
        _lastUnresolvedIndex[i] = -1;
    }
}

bool HealthMonitor::begin(bool enableWatchdog) {
    DEBUG_PRINTLN("HealthMonitor: Initializing...");
    
    if (enableWatchdog) {
        // ESP-IDF 5.x / Arduino ESP32 Core 3.x may already have TWDT initialized
        // Try to reconfigure it, or just add our task to existing watchdog
        esp_task_wdt_config_t wdt_config = {
            .timeout_ms = WATCHDOG_TIMEOUT_MS,
            .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // Watch all cores
            .trigger_panic = true
        };
        
        // Try to initialize - if already initialized, try reconfigure
        esp_err_t err = esp_task_wdt_init(&wdt_config);
        if (err == ESP_ERR_INVALID_STATE) {
            // Already initialized, try to reconfigure
            err = esp_task_wdt_reconfigure(&wdt_config);
            if (err != ESP_OK) {
                DEBUG_PRINTLN("HealthMonitor: Could not reconfigure watchdog, using defaults");
            }
        }
        
        // Add current task to watchdog (may already be subscribed)
        err = esp_task_wdt_add(NULL);
        if (err == ESP_ERR_INVALID_ARG) {
            // Task already subscribed, that's fine
            DEBUG_PRINTLN("HealthMonitor: Task already subscribed to watchdog");
        } else if (err != ESP_OK) {
            DEBUG_PRINTF("HealthMonitor: Failed to add task to watchdog: %d\n", err);
            _watchdogEnabled = false;
        } else {
            _watchdogEnabled = true;
            DEBUG_PRINTF("HealthMonitor: Watchdog enabled (%d ms)\n", WATCHDOG_TIMEOUT_MS);
        }
    }
    
    _lastCheckMs = millis();
    _status.uptime = 0;
    _status.minFreeHeap = ESP.getFreeHeap();
    
    DEBUG_PRINTLN("HealthMonitor: Initialization complete");
    return true;
}

void HealthMonitor::update() {
    uint32_t now = millis();
    
    // Feed watchdog
    if (_watchdogEnabled) {
        feedWatchdog();
    }
    
    // Periodic health check
    if (now - _lastCheckMs >= HEALTH_CHECK_INTERVAL_MS) {
        checkNow();
        _lastCheckMs = now;
    }
    
    _status.uptime = now / 1000;
}

void HealthMonitor::feedWatchdog() {
    if (_watchdogEnabled) {
        esp_err_t err = esp_task_wdt_reset();
        if (err != ESP_OK) {
            // Task not subscribed, try to add it
            if (err == ESP_ERR_INVALID_ARG || err == ESP_ERR_NOT_FOUND) {
                esp_task_wdt_add(NULL);
            }
        }
    }
}

void HealthMonitor::checkNow() {
    // Update memory stats
    _status.freeHeap = ESP.getFreeHeap();
    _status.freePsram = ESP.getFreePsram();
    
    if (_status.freeHeap < _status.minFreeHeap) {
        _status.minFreeHeap = _status.freeHeap;
    }
    
    // Run all health checks
    _status.gpsHealthy = checkGPSHealth();
    _status.motorAzHealthy = checkMotorHealth();
    _status.motorElHealthy = _status.motorAzHealthy;  // Combined check
    _status.wifiHealthy = checkNetworkHealth();
    _status.memoryHealthy = checkMemoryHealth();
    _status.cameraHealthy = true;  // No camera in this build
    
    // Overall health
    _status.overallHealthy = _status.gpsHealthy && 
                              _status.motorAzHealthy &&
                              _status.wifiHealthy && 
                              _status.memoryHealthy;
}

bool HealthMonitor::checkGPSHealth() {
    GPSData data = gps.getData();
    _status.gpsAge = data.age;
    
    // Check for stale data
    if (data.age > GPS_STALE_TIMEOUT_MS) {
        if (_status.lastIssue != HealthIssue::GPS_STALE) {
            logEvent(HealthIssue::GPS_STALE, "GPS data stale");
            if (_autoRecovery) recoverGPS();
        }
        return false;
    }
    
    // Check for fix (allow manual position)
    if (!data.valid && !gps.isUsingManualPosition()) {
        if (_status.lastIssue != HealthIssue::GPS_NO_FIX) {
            logEvent(HealthIssue::GPS_NO_FIX, "No GPS fix");
        }
        return false;
    }
    
    return true;
}

bool HealthMonitor::checkMotorHealth() {
    // CRITICAL: Skip ALL UART-based checks while motors are moving
    // UART communication to TMC2209 takes ~50-100ms and causes motion pauses
    if (stepperControl.isMoving()) {
        // Only check for stalls via DIAG pins (instant, doesn't require UART)
        if (stepperControl.getState(Axis::AZIMUTH) == MotorState::STALLED ||
            stepperControl.getState(Axis::ELEVATION) == MotorState::STALLED) {
            if (_status.lastIssue != HealthIssue::MOTOR_STALL) {
                logEvent(HealthIssue::MOTOR_STALL, "Motor stall detected");
                if (_autoRecovery) recoverMotor();
            }
            return false;
        }
        return true;  // Assume healthy during motion - no UART queries
    }
    
    // Motors are idle - safe to do UART checks
    
    // Skip frequent UART-based checks if drivers aren't responding
    // This prevents the UART timeout from causing issues
    // When UART fails, we only retest every ~60 seconds (12 * 5s interval)
    if (!_motorUartWorking) {
        _motorUartFailCount++;
        if (_motorUartFailCount < 12) {
            // Skip UART test - motors still work via STEP/DIR
            return true;  // Assume OK, skip UART test
        }
        _motorUartFailCount = 0;  // Reset and try UART again
    }
    
    // Check driver UART communication (only when idle)
    if (!stepperControl.driversConnected()) {
        _motorUartWorking = false;
        if (_status.lastIssue != HealthIssue::MOTOR_COMM_FAIL) {
            logEvent(HealthIssue::MOTOR_COMM_FAIL, "Motor driver communication failed");
            // Don't attempt recovery - motors work fine via STEP/DIR without UART
        }
        // Return true - motors still work via STEP/DIR even without UART
        return true;
    }
    
    // UART is working
    _motorUartWorking = true;
    _motorUartFailCount = 0;
    
    // Check for stalls
    if (stepperControl.getState(Axis::AZIMUTH) == MotorState::STALLED ||
        stepperControl.getState(Axis::ELEVATION) == MotorState::STALLED) {
        if (_status.lastIssue != HealthIssue::MOTOR_STALL) {
            logEvent(HealthIssue::MOTOR_STALL, "Motor stall detected");
            if (_autoRecovery) recoverMotor();
        }
        return false;
    }
    
    // Get driver temperatures (from DRV_STATUS) - only if UART working and idle
    uint32_t azStatus = stepperControl.getDriverStatus(Axis::AZIMUTH);
    uint32_t elStatus = stepperControl.getDriverStatus(Axis::ELEVATION);
    
    // Check overtemperature flags (bit 1 = ot, bit 2 = otpw)
    if ((azStatus & 0x06) || (elStatus & 0x06)) {
        if (_status.lastIssue != HealthIssue::MOTOR_OVERTEMP) {
            logEvent(HealthIssue::MOTOR_OVERTEMP, "Motor driver overtemperature");
            if (_autoRecovery) {
                stepperControl.stop();
                stepperControl.setEnabled(false);
            }
        }
        return false;
    }
    
    return true;
}

bool HealthMonitor::checkNetworkHealth() {
    _status.wifiRSSI = wifiManager.getRSSI();
    
    // Check connection
    if (!wifiManager.isConnected() && !wifiManager.isAPMode()) {
        if (_status.lastIssue != HealthIssue::WIFI_DISCONNECTED) {
            logEvent(HealthIssue::WIFI_DISCONNECTED, "WiFi disconnected");
            if (_autoRecovery) recoverNetwork();
        }
        return false;
    }
    
    // Check signal strength (only in station mode)
    if (wifiManager.isConnected() && _status.wifiRSSI < WIFI_MIN_RSSI) {
        if (_status.lastIssue != HealthIssue::WIFI_WEAK_SIGNAL) {
            logEvent(HealthIssue::WIFI_WEAK_SIGNAL, "Weak WiFi signal");
        }
        // Don't fail health for weak signal, just warn
    }
    
    return true;
}

bool HealthMonitor::checkMemoryHealth() {
    // Check heap
    if (_status.freeHeap < HEAP_MIN_FREE_BYTES) {
        if (_status.lastIssue != HealthIssue::HEAP_LOW) {
            logEvent(HealthIssue::HEAP_LOW, "Low heap memory");
        }
        return false;
    }
    
    // Check for fragmentation (largest free block vs total free)
    size_t largestBlock = ESP.getMaxAllocHeap();
    if (largestBlock < _status.freeHeap / 2) {
        if (_status.lastIssue != HealthIssue::HEAP_FRAGMENTED) {
            logEvent(HealthIssue::HEAP_FRAGMENTED, "Heap fragmentation detected");
        }
        // Don't fail for fragmentation, just warn
    }
    
    return true;
}

bool HealthMonitor::checkCameraHealth() {
    // Camera not available in this build
    return true;
}

void HealthMonitor::attemptRecovery(HealthIssue issue) {
    switch (issue) {
        case HealthIssue::GPS_STALE:
        case HealthIssue::GPS_NO_FIX:
            recoverGPS();
            break;
            
        case HealthIssue::MOTOR_STALL:
        case HealthIssue::MOTOR_COMM_FAIL:
        case HealthIssue::MOTOR_OVERTEMP:
            recoverMotor();
            break;
            
        case HealthIssue::WIFI_DISCONNECTED:
        case HealthIssue::WIFI_WEAK_SIGNAL:
            recoverNetwork();
            break;
            
        case HealthIssue::CAMERA_FAIL:
            recoverCamera();
            break;
            
        default:
            break;
    }
}

void HealthMonitor::recoverGPS() {
    DEBUG_PRINTLN("HealthMonitor: Attempting GPS recovery...");
    // GPS recovery is mostly passive - just wait for fix
    // Could send reset command to GPS module if supported
}

void HealthMonitor::recoverMotor() {
    DEBUG_PRINTLN("HealthMonitor: Attempting motor recovery...");
    
    // Stop motion
    stepperControl.stop();
    
    // Brief disable then re-enable
    stepperControl.setEnabled(false);
    delay(500);
    stepperControl.setEnabled(true);
    
    logRecovery(HealthIssue::MOTOR_STALL);
}

void HealthMonitor::recoverNetwork() {
    DEBUG_PRINTLN("HealthMonitor: Attempting network recovery...");
    // WiFiManager handles reconnection automatically
}

void HealthMonitor::recoverCamera() {
    // Camera not available in this build
    DEBUG_PRINTLN("HealthMonitor: Camera recovery not available (no camera)");
}

void HealthMonitor::logEvent(HealthIssue issue, const char* description) {
    HealthEvent event;
    event.timestamp = millis() / 1000;
    event.issue = issue;
    event.resolved = false;
    strncpy(event.description, description, sizeof(event.description) - 1);
    event.description[sizeof(event.description) - 1] = '\0';
    
    // Add to circular buffer
    _eventLog[_eventHead] = event;
    
    // Track this as the last unresolved event of this type for O(1) recovery lookup
    int issueIndex = static_cast<int>(issue);
    if (issueIndex >= 0 && issueIndex < NUM_ISSUE_TYPES) {
        _lastUnresolvedIndex[issueIndex] = _eventHead;
    }
    
    _eventHead = (_eventHead + 1) % MAX_EVENTS;
    if (_eventCount < MAX_EVENTS) _eventCount++;
    
    // Update status
    _status.lastIssue = issue;
    _status.issueCount++;
    _status.lastIssueTime = event.timestamp;
    
    DEBUG_PRINTF("HealthMonitor: %s\n", description);
}

void HealthMonitor::logRecovery(HealthIssue issue) {
    // O(1) lookup using issue index instead of linear search
    int issueIndex = static_cast<int>(issue);
    if (issueIndex >= 0 && issueIndex < NUM_ISSUE_TYPES) {
        int idx = _lastUnresolvedIndex[issueIndex];
        if (idx >= 0 && idx < MAX_EVENTS) {
            if (_eventLog[idx].issue == issue && !_eventLog[idx].resolved) {
                _eventLog[idx].resolved = true;
                _lastUnresolvedIndex[issueIndex] = -1;  // Clear the index
            }
        }
    }
    
    DEBUG_PRINTF("HealthMonitor: Recovered from issue %d\n", (int)issue);
}

int HealthMonitor::getEventLog(HealthEvent* events, int maxEvents) {
    int count = min(_eventCount, maxEvents);
    if (count == 0) return 0;
    
    // Calculate start index in circular buffer
    int startIdx = (_eventHead - _eventCount + MAX_EVENTS) % MAX_EVENTS;
    
    // Check if events are contiguous in buffer (no wrap-around)
    if (startIdx + count <= MAX_EVENTS) {
        // Single contiguous block - use memcpy for O(1) copy
        memcpy(events, &_eventLog[startIdx], count * sizeof(HealthEvent));
    } else {
        // Wrap-around case - copy in two parts
        int firstPart = MAX_EVENTS - startIdx;
        memcpy(events, &_eventLog[startIdx], firstPart * sizeof(HealthEvent));
        memcpy(&events[firstPart], &_eventLog[0], (count - firstPart) * sizeof(HealthEvent));
    }
    
    return count;
}

void HealthMonitor::clearEventLog() {
    _eventHead = 0;
    _eventCount = 0;
    _status.issueCount = 0;
    
    // Reset issue index array
    for (int i = 0; i < NUM_ISSUE_TYPES; i++) {
        _lastUnresolvedIndex[i] = -1;
    }
}

String HealthMonitor::getStatusString() const {
    String status = "Health: ";
    
    if (_status.overallHealthy) {
        status += "OK";
    } else {
        status += "ISSUES - ";
        if (!_status.gpsHealthy) status += "GPS ";
        if (!_status.motorAzHealthy) status += "Motor ";
        if (!_status.wifiHealthy) status += "WiFi ";
        if (!_status.memoryHealthy) status += "Memory ";
    }
    
    status += " | Heap: " + String(_status.freeHeap / 1024) + "KB";
    status += " | Up: " + String(_status.uptime / 3600) + "h";
    
    return status;
}
