/**
 * @file stepper_control.h
 * @brief TMC2209 stepper motor control for Az/El tracking
 * 
 * Provides motion control for both azimuth and elevation axes using
 * TMC2209 stepper drivers in UART mode with StallGuard homing.
 */

#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <Arduino.h>
#include <TMCStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config.h"

/**
 * @brief RAII mutex guard for thread-safe operations
 * 
 * Automatically acquires mutex on construction and releases on destruction,
 * ensuring proper cleanup even if exceptions occur or function returns early.
 */
class MutexGuard {
public:
    explicit MutexGuard(SemaphoreHandle_t mutex, TickType_t timeout = portMAX_DELAY)
        : _mutex(mutex), _acquired(false) {
        if (_mutex) {
            _acquired = (xSemaphoreTake(_mutex, timeout) == pdTRUE);
        }
    }
    
    ~MutexGuard() {
        release();
    }
    
    // Non-copyable
    MutexGuard(const MutexGuard&) = delete;
    MutexGuard& operator=(const MutexGuard&) = delete;
    
    // Check if mutex was successfully acquired
    bool acquired() const { return _acquired; }
    
    // Early release (idempotent)
    void release() {
        if (_acquired && _mutex) {
            xSemaphoreGive(_mutex);
            _acquired = false;
        }
    }
    
private:
    SemaphoreHandle_t _mutex;
    bool _acquired;
};

// Motor state enumeration
enum class MotorState {
    IDLE,           // Not moving
    MOVING,         // Moving to target
    HOMING,         // Sensorless homing
    STALLED,        // Stall detected
    ERROR           // Error state
};

// Axis enumeration
enum class Axis {
    AZIMUTH,
    ELEVATION
};

// Motion profile for smooth acceleration
struct MotionProfile {
    float targetPosition;       // Target position in degrees
    float currentPosition;      // Current position in degrees
    float currentSpeed;         // Current speed in deg/s
    float maxSpeed;             // Maximum speed in deg/s
    float acceleration;         // Acceleration in deg/s²
    bool isMoving;              // Currently in motion
    int32_t targetSteps;        // Target in steps
    int32_t currentSteps;       // Current step count
    
    // Cached step interval to avoid per-update FPU calculation
    uint32_t cachedStepInterval; // Step interval in microseconds
    float cachedSpeed;           // Speed value when interval was calculated
};

/**
 * @class StepperControl
 * @brief Manages both azimuth and elevation stepper motors
 */
class StepperControl {
public:
    StepperControl();
    
    /**
     * @brief Initialize stepper drivers and GPIO
     * @return true if initialization successful
     */
    bool begin();
    
    /**
     * @brief Update motor positions (call frequently from main loop)
     * Should be called at least every 1ms for smooth motion
     */
    void update();
    
    /**
     * @brief Move azimuth to absolute position
     * @param degrees Target position in degrees
     * @param speed Speed in degrees/second (0 = use default)
     */
    void moveAzimuthTo(float degrees, float speed = 0);
    
    /**
     * @brief Move elevation to absolute position  
     * @param degrees Target position in degrees (0-90)
     * @param speed Speed in degrees/second (0 = use default)
     */
    void moveElevationTo(float degrees, float speed = 0);
    
    /**
     * @brief Move both axes simultaneously
     * @param azDegrees Azimuth target
     * @param elDegrees Elevation target
     * @param speed Speed in degrees/second
     */
    void moveTo(float azDegrees, float elDegrees, float speed = 0);
    
    /**
     * @brief Stop all motion immediately
     */
    void stop();
    
    /**
     * @brief Emergency stop - cuts power to motors
     */
    void emergencyStop();
    
    /**
     * @brief Enable/disable motors
     * @param enabled true to enable, false to disable
     */
    void setEnabled(bool enabled);
    
    /**
     * @brief Check if motors are enabled
     */
    bool isEnabled() const { return _enabled; }
    
    /**
     * @brief Home both axes using StallGuard
     * @return true if homing successful
     */
    bool home();
    
    /**
     * @brief Home single axis
     * @param axis Axis to home
     * @return true if successful
     */
    bool homeAxis(Axis axis);
    
    /**
     * @brief Set current position as home (0,0)
     */
    void setHome();
    
    /**
     * @brief Move to park position
     */
    void park();
    
    /**
     * @brief Get current azimuth position
     * @return Position in degrees
     */
    float getAzimuth() const;
    
    /**
     * @brief Get current elevation position
     * @return Position in degrees
     */
    float getElevation() const;
    
    /**
     * @brief Get target azimuth position
     */
    float getTargetAzimuth() const { return _azProfile.targetPosition; }
    
    /**
     * @brief Get target elevation position
     */
    float getTargetElevation() const { return _elProfile.targetPosition; }
    
    /**
     * @brief Check if any axis is currently moving
     */
    bool isMoving() const;
    
    /**
     * @brief Check if azimuth is moving
     */
    bool isAzimuthMoving() const { return _azProfile.isMoving; }
    
    /**
     * @brief Check if elevation is moving
     */
    bool isElevationMoving() const { return _elProfile.isMoving; }
    
    /**
     * @brief Get motor state for an axis
     */
    MotorState getState(Axis axis) const;
    
    /**
     * @brief Apply calibration offset
     * @param azOffset Azimuth offset in degrees
     * @param elOffset Elevation offset in degrees
     */
    void setCalibrationOffset(float azOffset, float elOffset);
    
    /**
     * @brief Get azimuth offset
     */
    float getAzimuthOffset() const { return _azOffset; }
    
    /**
     * @brief Get elevation offset
     */
    float getElevationOffset() const { return _elOffset; }
    
    /**
     * @brief Set position directly (for restoring from NVS)
     * @param azDegrees Azimuth position
     * @param elDegrees Elevation position
     */
    void setPosition(float azDegrees, float elDegrees);
    
    /**
     * @brief Set motor current
     * @param runCurrent_mA Run current in milliamps
     * @param holdCurrent_mA Hold current in milliamps
     */
    void setMotorCurrent(uint16_t runCurrent_mA, uint16_t holdCurrent_mA);
    
    /**
     * @brief Get driver status
     * @param axis Which axis
     * @return TMC2209 status register value
     */
    uint32_t getDriverStatus(Axis axis);
    
    /**
     * @brief Check for driver communication
     * @return true if both drivers responding
     */
    bool driversConnected();

private:
    // TMC2209 driver instances
    TMC2209Stepper* _tmcAz;
    TMC2209Stepper* _tmcEl;
    
    // Hardware serial for TMC UART
    HardwareSerial* _tmcSerial;
    
    // Motion profiles
    MotionProfile _azProfile;
    MotionProfile _elProfile;
    
    // Motor states
    MotorState _azState;
    MotorState _elState;
    
    // Calibration offsets
    float _azOffset;
    float _elOffset;
    
    // Enable state
    bool _enabled;
    
    // Mutex for thread-safe access to motion profiles
    SemaphoreHandle_t _profileMutex;
    
    // Last update time
    uint32_t _lastUpdateMicros;
    
    // Step timing
    uint32_t _azLastStepMicros;
    uint32_t _elLastStepMicros;
    
    // Internal methods
    void initDriver(TMC2209Stepper* driver, uint8_t address);
    void updateAxis(MotionProfile& profile, uint8_t stepPin, uint8_t dirPin, 
                    uint32_t& lastStepMicros, float minPos, float maxPos, bool reversed);
    float stepsToDegrees(int32_t steps);
    int32_t degreesToSteps(float degrees);
    void pulseStep(uint8_t stepPin);
    bool detectStall(Axis axis);
    float normalizeAzimuth(float degrees);
    float shortestPath(float current, float target);
};

// Global instance
extern StepperControl stepperControl;

#endif // STEPPER_CONTROL_H
