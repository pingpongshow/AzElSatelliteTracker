/**
 * @file config.h
 * @brief Main configuration file for Az/El Satellite Tracker
 * 
 * This file contains all pin definitions, constants, and configuration
 * parameters for the satellite tracker system.
 * 
 * Hardware:
 * - ESP32-S3 DevKit
 * - TMC2209 stepper drivers (x2)
 * - NEMA 17 5.18:1 geared steppers (x2)
 * - GPS module (NMEA)
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =============================================================================
// VERSION INFO
// =============================================================================
#define FIRMWARE_VERSION "1.1.0"
#define DEVICE_NAME "SatTracker"
#define DEVICE_MODEL "AzEl-S3"

// =============================================================================
// WIFI CONFIGURATION
// =============================================================================
#define WIFI_CONNECT_TIMEOUT_MS     15000
#define WIFI_RECONNECT_INTERVAL_MS  30000

// Access Point configuration (fallback mode)
#define AP_SSID                     "SatTracker-AP"
#define AP_PASSWORD                 "satellite123"
#define AP_CHANNEL                  1       // Channel 1 for best compatibility
#define AP_MAX_CONNECTIONS          4

// mDNS hostname
#define MDNS_HOSTNAME               "satellite-tracker"

// =============================================================================
// WEB SERVER CONFIGURATION
// =============================================================================
#define WEB_SERVER_PORT             80
#define WEBSOCKET_PORT              81
#define API_PREFIX                  "/api"

// =============================================================================
// JSON DOCUMENT SIZES
// =============================================================================
// StaticJsonDocument sizes for stack allocation (avoids heap fragmentation)
// Use these for frequently-called functions with known document sizes

#define JSON_SIZE_TINY              64      // Simple responses: {"ok":true}
#define JSON_SIZE_SMALL             128     // Error/OK responses with message
#define JSON_SIZE_MEDIUM            256     // Status updates, pass info
#define JSON_SIZE_LARGE             512     // TLE entries, detailed status
#define JSON_SIZE_XLARGE            1024    // Lists, API responses
#define JSON_SIZE_XXLARGE           2048    // Large lists, config dumps

// =============================================================================
// ROTCTLD CONFIGURATION (Hamlib protocol)
// =============================================================================
#define ROTCTLD_PORT                4533
#define ROTCTLD_MAX_CLIENTS         2
#define ROTCTLD_BUFFER_SIZE         256

// =============================================================================
// TMC2209 STEPPER DRIVER PINS
// =============================================================================
// Azimuth motor
#define TMC_AZ_STEP_PIN             38
#define TMC_AZ_DIR_PIN              39
#define TMC_AZ_DIAG_PIN             40      // StallGuard diagnostic output

// Elevation motor  
#define TMC_EL_STEP_PIN             41
#define TMC_EL_DIR_PIN              42
#define TMC_EL_DIAG_PIN             2       // StallGuard diagnostic output

// Shared pins
#define TMC_EN_PIN                  1       // Shared enable (active LOW)
#define TMC_UART_TX_PIN             47      // UART TX to both drivers
#define TMC_UART_RX_PIN             13      // UART RX from both drivers

// TMC2209 UART addresses (directly wired to MS1/MS2)
#define TMC_AZ_ADDRESS              0b00    // MS1=LOW, MS2=LOW
#define TMC_EL_ADDRESS              0b01    // MS1=HIGH, MS2=LOW

// =============================================================================
// TMC2209 MOTOR CONFIGURATION
// =============================================================================
#define TMC_SERIAL_BAUD             115200
#define TMC_R_SENSE                 0.11f   // Sense resistor value (ohms)

// Motor current settings (RMS milliamps)
// NOTE: Geared steppers need more torque - increase if motor stalls
#define MOTOR_RUN_CURRENT_MA        1200    // Running current (was 800, increased for geared motors)
#define MOTOR_HOLD_CURRENT_MA       600     // Holding current
#define MOTOR_HOLD_MULTIPLIER       0.5f    // Hold current = run * multiplier

// Microstepping (256 is max for TMC2209)
// Lower values = more torque but rougher motion
#define TMC_MICROSTEPS              16      // Reduced from 32 for more torque

// StallGuard threshold (0-255, lower = more sensitive)
#define TMC_STALL_THRESHOLD         50

// =============================================================================
// STEPPER MOTOR MECHANICAL CONFIGURATION
// =============================================================================
#define STEPS_PER_REV               200     // 1.8° stepper
#define GEAR_RATIO                  5.18f   // 5.18:1 geared stepper

// Calculated steps per degree
// (STEPS_PER_REV * GEAR_RATIO * TMC_MICROSTEPS) / 360
#define STEPS_PER_DEGREE            ((STEPS_PER_REV * GEAR_RATIO * TMC_MICROSTEPS) / 360.0f)

// Speed limits (degrees per second)
// NOTE: Reduce speeds if motor stalls - geared steppers are torque-limited at high speeds
#define MAX_SLEW_SPEED_DEG_S        15.0f   // Fast repositioning (was 30)
#define TRACKING_SPEED_DEG_S        3.0f    // Normal tracking (was 5)
#define HOMING_SPEED_DEG_S          5.0f    // Homing speed (was 10)

// Acceleration (degrees per second squared)
// Lower = smoother startup but slower response
#define ACCELERATION_DEG_S2         8.0f    // Reduced from 20 for smoother ramp-up

// =============================================================================
// AXIS LIMITS
// =============================================================================
// Azimuth limits (continuous rotation possible, but limit for safety)
#define AZ_MIN_DEG                  -180.0f
#define AZ_MAX_DEG                  540.0f  // Allow 1.5 rotations for cable wrap

// Elevation limits
#define EL_MIN_DEG                  0.0f
#define EL_MAX_DEG                  90.0f

// Zenith avoidance threshold
#define ZENITH_THRESHOLD_DEG        87.0f   // Start flip planning above this

// Park position
#define PARK_AZ_DEG                 0.0f
#define PARK_EL_DEG                 45.0f

// =============================================================================
// MOTOR DIRECTION REVERSAL
// =============================================================================
// Set to true to reverse motor direction (if motor moves opposite to expected)
#define AZ_MOTOR_REVERSED           false    // Set to true to reverse azimuth
#define EL_MOTOR_REVERSED           true    // Set to true to reverse elevation

// =============================================================================
// GPS CONFIGURATION
// =============================================================================
#define GPS_SERIAL_NUM              1       // Hardware serial port number
#define GPS_RX_PIN                  14      // GPS TX -> ESP RX
#define GPS_TX_PIN                  21      // ESP TX -> GPS RX  
#define GPS_BAUD                    9600

// GPS timeout and update intervals
#define GPS_FIX_TIMEOUT_MS          120000  // 2 minutes to get fix
#define GPS_UPDATE_INTERVAL_MS      1000    // Position update rate

// Default position (used before GPS fix)
// Set this to your approximate location
#define DEFAULT_LATITUDE            40.7128f    // New York City
#define DEFAULT_LONGITUDE           -74.0060f
#define DEFAULT_ALTITUDE            10.0f       // meters

// =============================================================================
// TRACKING ENGINE CONFIGURATION
// =============================================================================
// TLE update interval (milliseconds)
#define TLE_UPDATE_INTERVAL_MS      86400000    // 24 hours
#define TLE_FETCH_TIMEOUT_MS        30000       // 30 seconds

// TLE sources
#define TLE_SOURCE_CELESTRAK        "https://celestrak.org/NORAD/elements/gp.php"
#define TLE_SOURCE_AMATEUR          "https://celestrak.org/NORAD/elements/gp.php?GROUP=amateur&FORMAT=tle"

// Maximum stored TLEs
#define MAX_STORED_TLES             20

// Position calculation interval during tracking
#define TRACKING_UPDATE_INTERVAL_MS 100     // 10 Hz update rate

// Look-ahead time for zenith flip prediction (seconds)
#define ZENITH_LOOKAHEAD_SEC        10

// =============================================================================
// NVS STORAGE KEYS
// =============================================================================
#define NVS_NAMESPACE               "sattracker"
#define NVS_KEY_WIFI_SSID           "wifi_ssid"
#define NVS_KEY_WIFI_PASS           "wifi_pass"
#define NVS_KEY_AZ_OFFSET           "az_offset"
#define NVS_KEY_EL_OFFSET           "el_offset"
#define NVS_KEY_AZ_POSITION         "az_pos"
#define NVS_KEY_EL_POSITION         "el_pos"
#define NVS_KEY_LATITUDE            "latitude"
#define NVS_KEY_LONGITUDE           "longitude"
#define NVS_KEY_ALTITUDE            "altitude"
#define NVS_KEY_TRACKING_SAT        "track_sat"

// =============================================================================
// TIMING AND SCHEDULING
// =============================================================================
#define MAIN_LOOP_INTERVAL_MS       10      // Main loop target interval
#define STATUS_UPDATE_INTERVAL_MS   500     // Status broadcast to WebSocket
#define POSITION_SAVE_INTERVAL_MS   60000   // Save position to NVS

// =============================================================================
// DEBUG CONFIGURATION
// =============================================================================
#define DEBUG_SERIAL                Serial
#define DEBUG_BAUD                  115200

#ifdef CORE_DEBUG_LEVEL
    #if CORE_DEBUG_LEVEL >= 3
        #define DEBUG_PRINT(x)      DEBUG_SERIAL.print(x)
        #define DEBUG_PRINTLN(x)    DEBUG_SERIAL.println(x)
        #define DEBUG_PRINTF(...)   DEBUG_SERIAL.printf(__VA_ARGS__)
    #else
        // When debug is disabled, macros still consume their arguments to avoid
        // "unused variable" warnings. The if(0) branch is optimized away by the
        // compiler, but variables referenced in the arguments are marked as used.
        #define DEBUG_PRINT(x)      do { (void)(x); } while(0)
        #define DEBUG_PRINTLN(x)    do { (void)(x); } while(0)
        #define DEBUG_PRINTF(...)   do { if(0) { DEBUG_SERIAL.printf(__VA_ARGS__); } } while(0)
    #endif
#else
    // When CORE_DEBUG_LEVEL is not defined, still consume arguments
    #define DEBUG_PRINT(x)      do { (void)(x); } while(0)
    #define DEBUG_PRINTLN(x)    do { (void)(x); } while(0)
    #define DEBUG_PRINTF(...)   do { if(0) { Serial.printf(__VA_ARGS__); } } while(0)
#endif

// =============================================================================
// TASK PRIORITIES (FreeRTOS)
// =============================================================================
#define TASK_PRIORITY_STEPPER       5       // Highest - motor control
#define TASK_PRIORITY_GPS           3       // Medium - GPS parsing
#define TASK_PRIORITY_TRACKING      3       // Medium - position calculation
#define TASK_PRIORITY_NETWORK       2       // Lower - web/API

// Task stack sizes (words)
#define TASK_STACK_STEPPER          4096
#define TASK_STACK_GPS              2048
#define TASK_STACK_TRACKING         4096
#define TASK_STACK_NETWORK          8192

// =============================================================================
// MULTI-PASS PREDICTION & SCHEDULING
// =============================================================================
#define MAX_PREDICTED_PASSES        20      // Maximum passes to predict
#define MAX_SCHEDULED_PASSES        10      // Maximum scheduled auto-track passes
#define PASS_SEARCH_HOURS           48      // Hours to search ahead
#define PASS_MIN_ELEVATION          5.0f    // Minimum max elevation for valid pass
#define PRE_POSITION_SECONDS        120     // Seconds before AOS to pre-position
#define PRE_POSITION_ELEVATION      5.0f    // Elevation to start at for pre-position

// =============================================================================
// DOPPLER CONFIGURATION
// =============================================================================
#define SPEED_OF_LIGHT_KMS          299792.458  // km/s

// =============================================================================
// HEALTH MONITORING & WATCHDOG
// =============================================================================
#define WATCHDOG_TIMEOUT_MS         30000   // 30 second watchdog
#define HEALTH_CHECK_INTERVAL_MS    5000    // Check health every 5 seconds
#define GPS_STALE_TIMEOUT_MS        10000   // GPS data stale after 10 seconds
#define WIFI_MIN_RSSI               -80     // Minimum acceptable RSSI
#define HEAP_MIN_FREE_BYTES         20000   // Minimum free heap warning
#define MOTOR_MAX_TEMP_C            80      // TMC2209 temperature limit

// =============================================================================
// EXTERNAL API INTEGRATION
// =============================================================================
#define N2YO_API_URL                "https://api.n2yo.com/rest/v1/satellite"
#define N2YO_API_TIMEOUT_MS         15000
#define SATNOGS_API_URL             "https://db.satnogs.org/api"
#define CELESTRAK_API_URL           "https://celestrak.org/NORAD/elements/gp.php"

// =============================================================================
// UDP BROADCAST (GPredict compatible)
// =============================================================================
#define UDP_BROADCAST_PORT          4532
#define UDP_BROADCAST_INTERVAL_MS   100     // 10 Hz update rate
#define UDP_MULTICAST_ADDR          "239.255.0.1"

// =============================================================================
// SD CARD CONFIGURATION
// =============================================================================
#define SD_CS_PIN                   -1      // Set to actual CS pin if using SD
#define SD_SPI_FREQ_MHZ             20
#define TLE_STORAGE_DIR             "/tle"
#define PASS_LOG_DIR                "/logs"
#define CONFIG_BACKUP_FILE          "/config.json"

// =============================================================================
// ANTENNA PATTERN COMPENSATION
// =============================================================================
#define DEFAULT_AZ_BEAM_OFFSET      0.0f    // Degrees
#define DEFAULT_EL_BEAM_OFFSET      0.0f    // Degrees
#define DEFAULT_BEAMWIDTH           30.0f   // Degrees (3dB beamwidth)

// =============================================================================
// EME (MOON BOUNCE) MODE
// =============================================================================
#define EME_TRACKING_RATE           0.5f    // Slower tracking for EME
#define EME_MOON_RADIUS_KM          1737.4  // For libration calculations
#define EME_AVG_DISTANCE_KM         384400  // Average Earth-Moon distance

// =============================================================================
// GEOSTATIONARY SATELLITE MODE
// =============================================================================
#define GEO_ALTITUDE_KM             35786   // Geostationary orbit altitude
#define GEO_POSITION_TOLERANCE      0.1f    // Degrees - repoint if drift exceeds
#define GEO_UPDATE_INTERVAL_MS      60000   // Check geo position every minute

// =============================================================================
// STEPPER MOTION CONTROL CONSTANTS
// =============================================================================
#define STEPPER_DT_CAP_SEC          0.01f   // Maximum delta-time for acceleration calc (10ms)
#define STEPPER_MIN_STEP_INTERVAL_US 20     // Minimum step interval in microseconds (50kHz max)
#define STEPPER_MIN_SPEED_DEG_S     0.5f    // Minimum speed to prevent stalling (increased for faster starts)

// =============================================================================
// ROTCTLD PROTOCOL CONSTANTS
// =============================================================================
#define ROTCTLD_MOVE_AMOUNT_DEG     10.0f   // Default move amount for directional commands
#define ROTCTLD_DIR_UP              2       // Hamlib direction bitmask: up
#define ROTCTLD_DIR_DOWN            4       // Hamlib direction bitmask: down
#define ROTCTLD_DIR_LEFT            8       // Hamlib direction bitmask: left
#define ROTCTLD_DIR_RIGHT           16      // Hamlib direction bitmask: right
#define ROTCTLD_MIN_SPEED_DEG_S     0.1f    // Minimum speed for move commands
#define ROTCTLD_MAX_SPEED_DEG_S     MAX_SLEW_SPEED_DEG_S  // Maximum speed (use global max)

// =============================================================================
// TIME AND TRACKING CONSTANTS
// =============================================================================
#define GPS_EXTRAPOLATION_WARN_SEC  600     // Warn after extrapolating time for 10 minutes

#endif // CONFIG_H
