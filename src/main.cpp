/**
 * @file main.cpp
 * @brief Main entry point for Az/El Satellite Tracker
 * 
 * ESP32-S3 based satellite antenna tracker with:
 * - TMC2209 stepper drivers for azimuth and elevation
 * - GPS for location and time
 * - SGP4 orbit propagation for satellite tracking
 * - rotctld protocol for GPredict compatibility
 * - Web interface for configuration and manual control
 * 
 * @author Your Name
 * @version 1.1.0
 * @date 2024
 * 
 * Hardware:
 * - ESP32-S3 DevKit
 * - TMC2209 stepper drivers (x2)
 * - NEMA 17 5.18:1 geared steppers (x2)
 * - GPS module (NMEA)
 */

#include <Arduino.h>
#include "config.h"
#include "wifi_manager.h"
#include "web_server.h"
#include "rotctld_server.h"
#include "gps.h"
#include "stepper_control.h"
#include "tracking_engine.h"
#include "tle_manager.h"
#include "nvs_storage.h"
#include "health_monitor.h"
#include "sd_storage.h"
#include "udp_broadcast.h"
#include "external_api.h"

// Task handles for FreeRTOS
TaskHandle_t stepperTaskHandle = NULL;
TaskHandle_t networkTaskHandle = NULL;

// Timing variables
uint32_t lastStatusPrint = 0;
uint32_t lastPositionSave = 0;
uint32_t lastMainLoop = 0;

// Position tracking for wear-leveling
float lastSavedAzimuth = 0;
float lastSavedElevation = 0;
const float POSITION_SAVE_THRESHOLD = 0.5f;  // Only save if moved > 0.5 degrees

// Forward declarations
void stepperTask(void* parameter);
void networkTask(void* parameter);
void printStatus();
void loadSavedState();

/**
 * @brief Arduino setup function
 */
void setup() {
    // Initialize serial for debugging
    Serial.begin(DEBUG_BAUD);
    delay(1000);
    
    Serial.println();
    Serial.println("==========================================");
    Serial.println("  Az/El Satellite Tracker");
    Serial.printf("  Version: %s\n", FIRMWARE_VERSION);
    Serial.printf("  Model: %s %s\n", DEVICE_NAME, DEVICE_MODEL);
    Serial.println("==========================================");
    Serial.println();
    
    // Initialize NVS storage first
    Serial.println("[INIT] NVS Storage...");
    if (!nvsStorage.begin()) {
        Serial.println("[ERROR] NVS initialization failed!");
    }
    
    // Initialize GPS
    Serial.println("[INIT] GPS...");
    if (!gps.begin()) {
        Serial.println("[ERROR] GPS initialization failed!");
    }
    
    // Initialize stepper control
    Serial.println("[INIT] Stepper Control...");
    if (!stepperControl.begin()) {
        Serial.println("[ERROR] Stepper initialization failed!");
        Serial.println("[ERROR] Check TMC2209 UART connections!");
    }
    
    // Initialize tracking engine
    Serial.println("[INIT] Tracking Engine...");
    if (!trackingEngine.begin()) {
        Serial.println("[ERROR] Tracking engine initialization failed!");
    }
    
    // Initialize TLE manager
    Serial.println("[INIT] TLE Manager...");
    if (!tleManager.begin()) {
        Serial.println("[ERROR] TLE manager initialization failed!");
    }
    
    // Initialize health monitor
    // NOTE: Watchdog disabled by default for initial testing/development
    // Set to 'true' once hardware is fully connected and working
    Serial.println("[INIT] Health Monitor...");
    if (!healthMonitor.begin(false)) {  // Watchdog disabled - change to true for production
        Serial.println("[WARN] Health monitor initialization failed");
    }
    
    // Initialize SD card storage (optional)
    Serial.println("[INIT] SD Card...");
    if (SD_CS_PIN >= 0) {
        if (!sdStorage.begin(SD_CS_PIN)) {
            Serial.println("[WARN] SD card not available - SD features disabled");
        }
    } else {
        Serial.println("[INFO] SD card not configured");
    }
    
    // Initialize WiFi
    Serial.println("[INIT] WiFi...");
    if (!wifiManager.begin()) {
        Serial.println("[ERROR] WiFi initialization failed!");
    }
    
    // Wait for WiFi connection or AP mode
    Serial.println("[INIT] Waiting for network...");
    uint32_t wifiTimeout = millis() + WIFI_CONNECT_TIMEOUT_MS;
    while (!wifiManager.isConnected() && !wifiManager.isAPMode() && millis() < wifiTimeout) {
        wifiManager.update();
        delay(100);
    }
    
    // Give the TCP/IP stack time to fully initialize
    // This prevents "tcp_alloc: Required to lock TCPIP core" assertion failures
    Serial.println("[INIT] Waiting for TCP/IP stack...");
    delay(500);
    
    // Initialize web server
    Serial.println("[INIT] Web Server...");
    if (!webServer.begin()) {
        Serial.println("[ERROR] Web server initialization failed!");
    }
    
    // Initialize rotctld server
    Serial.println("[INIT] rotctld Server...");
    if (!rotctldServer.begin()) {
        Serial.println("[ERROR] rotctld server initialization failed!");
    }
    
    // Initialize UDP broadcast
    Serial.println("[INIT] UDP Broadcast...");
    if (!udpBroadcast.begin(UDP_BROADCAST_PORT, UDPFormat::JSON)) {
        Serial.println("[WARN] UDP broadcast initialization failed");
    }
    
    // Load saved state
    loadSavedState();
    
    // Create stepper control task on Core 1 (high priority)
    xTaskCreatePinnedToCore(
        stepperTask,
        "StepperTask",
        TASK_STACK_STEPPER,
        NULL,
        TASK_PRIORITY_STEPPER,
        &stepperTaskHandle,
        1  // Core 1
    );
    
    // Create network task on Core 0
    xTaskCreatePinnedToCore(
        networkTask,
        "NetworkTask",
        TASK_STACK_NETWORK,
        NULL,
        TASK_PRIORITY_NETWORK,
        &networkTaskHandle,
        0  // Core 0
    );
    
    Serial.println();
    Serial.println("==========================================");
    Serial.println("  Initialization Complete!");
    Serial.println("==========================================");
    Serial.printf("  IP Address: %s\n", wifiManager.getIP().toString().c_str());
    Serial.printf("  Hostname: %s.local\n", MDNS_HOSTNAME);
    Serial.printf("  Web UI: http://%s/\n", wifiManager.getIP().toString().c_str());
    Serial.printf("  rotctld: %s:%d\n", wifiManager.getIP().toString().c_str(), ROTCTLD_PORT);
    Serial.printf("  UDP Broadcast: Port %d\n", UDP_BROADCAST_PORT);
    if (sdStorage.isAvailable()) {
        uint32_t totalMB, usedMB;
        sdStorage.getCardInfo(totalMB, usedMB);
        Serial.printf("  SD Card: %lu MB total, %lu MB used\n", totalMB, usedMB);
    }
    Serial.println("------------------------------------------");
    Serial.println("  Motor Configuration:");
    Serial.printf("    Run Current: %d mA\n", MOTOR_RUN_CURRENT_MA);
    Serial.printf("    Microsteps: %d\n", TMC_MICROSTEPS);
    Serial.printf("    Steps/degree: %.2f\n", STEPS_PER_DEGREE);
    Serial.printf("    Max Speed: %.1f °/s\n", MAX_SLEW_SPEED_DEG_S);
    Serial.printf("    Tracking Speed: %.1f °/s\n", TRACKING_SPEED_DEG_S);
    Serial.printf("    Acceleration: %.1f °/s²\n", ACCELERATION_DEG_S2);
    Serial.println("==========================================");
    Serial.println();
}

/**
 * @brief Arduino main loop
 */
void loop() {
    uint32_t now = millis();
    
    // Update GPS
    gps.update();
    
    // Update tracking engine
    trackingEngine.update();
    
    // Update health monitor
    healthMonitor.update();
    
    // Update UDP broadcast
    udpBroadcast.update();
    
    // Periodic position save - only when position changed significantly and motors idle
    // This reduces NVS flash wear (limited to ~100k write cycles per sector)
    if (now - lastPositionSave >= POSITION_SAVE_INTERVAL_MS) {
        float currentAz = stepperControl.getAzimuth();
        float currentEl = stepperControl.getElevation();
        
        // Only save if: not moving AND position changed significantly
        bool positionChanged = (abs(currentAz - lastSavedAzimuth) > POSITION_SAVE_THRESHOLD) ||
                               (abs(currentEl - lastSavedElevation) > POSITION_SAVE_THRESHOLD);
        
        if (!stepperControl.isMoving() && positionChanged) {
            nvsStorage.savePosition(currentAz, currentEl);
            lastSavedAzimuth = currentAz;
            lastSavedElevation = currentEl;
            DEBUG_PRINTF("Position saved: Az=%.2f, El=%.2f\n", currentAz, currentEl);
        }
        lastPositionSave = now;
    }
    
    // Periodic status print
    if (now - lastStatusPrint >= 5000) {
        printStatus();
        lastStatusPrint = now;
    }
    
    // Maintain loop timing
    uint32_t elapsed = millis() - now;
    if (elapsed < MAIN_LOOP_INTERVAL_MS) {
        delay(MAIN_LOOP_INTERVAL_MS - elapsed);
    }
}

/**
 * @brief Stepper control task (runs on Core 1)
 * 
 * High-priority task for smooth motor control
 */
void stepperTask(void* parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(1);  // 1ms update rate
    
    for (;;) {
        stepperControl.update();
        vTaskDelayUntil(&lastWakeTime, interval);
    }
}

/**
 * @brief Network task (runs on Core 0)
 * 
 * Handles WiFi, web server, and rotctld
 */
void networkTask(void* parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(10);  // 10ms update rate
    
    for (;;) {
        wifiManager.update();
        webServer.update();
        rotctldServer.update();
        vTaskDelayUntil(&lastWakeTime, interval);
    }
}

/**
 * @brief Print current system status to serial
 */
void printStatus() {
    Serial.println("--- Status ---");
    
    // Position with target info
    Serial.printf("Position: Az=%.2f° El=%.2f° %s\n",
                  stepperControl.getAzimuth(),
                  stepperControl.getElevation(),
                  stepperControl.isMoving() ? "[MOVING]" : "[IDLE]");
    
    if (stepperControl.isMoving()) {
        Serial.printf("  Target: Az=%.2f° El=%.2f°\n",
                      stepperControl.getTargetAzimuth(),
                      stepperControl.getTargetElevation());
    }
    
    // GPS
    if (gps.hasFix()) {
        Serial.printf("GPS: %.6f, %.6f, %.1fm (%d sats)\n",
                      gps.getLatitude(),
                      gps.getLongitude(),
                      gps.getAltitude(),
                      gps.getSatellites());
    } else {
        Serial.println("GPS: No fix");
    }
    
    // Tracking
    if (trackingEngine.hasTLE()) {
        Serial.printf("Tracking: %s ", trackingEngine.getSatelliteName().c_str());
        if (trackingEngine.isSatelliteVisible()) {
            SatellitePosition pos = trackingEngine.getPosition();
            Serial.printf("[VISIBLE Az=%.1f° El=%.1f°]\n", pos.azimuth, pos.elevation);
        } else {
            Serial.println("[Below horizon]");
        }
    }
    
    // Network
    Serial.printf("WiFi: %s | rotctld clients: %d\n",
                  wifiManager.getStatusString().c_str(),
                  rotctldServer.getClientCount());
    
    // Memory
    Serial.printf("Free heap: %d bytes | PSRAM: %d bytes\n",
                  ESP.getFreeHeap(),
                  ESP.getFreePsram());
    
    Serial.println();
}

/**
 * @brief Load saved state from NVS
 */
void loadSavedState() {
    TrackerConfig config;
    
    if (nvsStorage.loadConfig(config)) {
        // Restore calibration
        stepperControl.setCalibrationOffset(config.azimuthOffset, config.elevationOffset);
        
        // Restore position if valid
        if (config.positionValid) {
            Serial.printf("[RESTORE] Position: Az=%.2f° El=%.2f°\n",
                          config.lastAzimuth, config.lastElevation);
            stepperControl.setPosition(config.lastAzimuth, config.lastElevation);
            
            // Initialize tracking variables to prevent immediate re-save
            lastSavedAzimuth = config.lastAzimuth;
            lastSavedElevation = config.lastElevation;
        } else {
            Serial.println("[RESTORE] No saved position - homing required");
        }
        
        // Restore observer location if manual
        if (config.useManualLocation) {
            gps.setManualPosition(config.latitude, config.longitude, config.altitude);
            gps.setUseManualPosition(true);
            trackingEngine.setObserverLocation(config.latitude, config.longitude, 
                                                config.altitude);
        }
        
        // Restore motor settings
        stepperControl.setMotorCurrent(config.motorRunCurrent, config.motorHoldCurrent);
        
        Serial.println("[RESTORE] Configuration loaded from NVS");
    }
}
