/**
 * @file stepper_control.cpp
 * @brief TMC2209 stepper motor control implementation
 */

#include "stepper_control.h"

// Global instance
StepperControl stepperControl;

StepperControl::StepperControl() 
    : _tmcAz(nullptr)
    , _tmcEl(nullptr)
    , _tmcSerial(nullptr)
    , _azState(MotorState::IDLE)
    , _elState(MotorState::IDLE)
    , _azOffset(0.0f)
    , _elOffset(0.0f)
    , _enabled(false)
    , _profileMutex(nullptr)
    , _lastUpdateMicros(0)
    , _azLastStepMicros(0)
    , _elLastStepMicros(0)
{
    // Create mutex for thread-safe motion profile access
    _profileMutex = xSemaphoreCreateMutex();
    
    // Initialize motion profiles
    memset(&_azProfile, 0, sizeof(MotionProfile));
    memset(&_elProfile, 0, sizeof(MotionProfile));
    
    _azProfile.maxSpeed = TRACKING_SPEED_DEG_S;
    _elProfile.maxSpeed = TRACKING_SPEED_DEG_S;
    _azProfile.acceleration = ACCELERATION_DEG_S2;
    _elProfile.acceleration = ACCELERATION_DEG_S2;
    
    // Initialize step interval cache with invalid speed value to force recalculation
    // Using -1.0f ensures first comparison will trigger calculation
    _azProfile.cachedSpeed = -1.0f;
    _elProfile.cachedSpeed = -1.0f;
    _azProfile.cachedStepInterval = 50000;  // Default safe value (20 Hz)
    _elProfile.cachedStepInterval = 50000;
}

bool StepperControl::begin() {
    DEBUG_PRINTLN("StepperControl: Initializing...");
    
    // Configure GPIO pins
    pinMode(TMC_AZ_STEP_PIN, OUTPUT);
    pinMode(TMC_AZ_DIR_PIN, OUTPUT);
    pinMode(TMC_AZ_DIAG_PIN, INPUT);
    
    pinMode(TMC_EL_STEP_PIN, OUTPUT);
    pinMode(TMC_EL_DIR_PIN, OUTPUT);
    pinMode(TMC_EL_DIAG_PIN, INPUT);
    
    pinMode(TMC_EN_PIN, OUTPUT);
    digitalWrite(TMC_EN_PIN, HIGH);  // Disabled initially
    
    // Initialize TMC UART
    _tmcSerial = new HardwareSerial(2);
    _tmcSerial->begin(TMC_SERIAL_BAUD, SERIAL_8N1, TMC_UART_RX_PIN, TMC_UART_TX_PIN);
    
    // Debug: Print UART configuration
    Serial.printf("StepperControl: UART Config - TX_PIN=%d, RX_PIN=%d, Baud=%d\n", 
                  TMC_UART_TX_PIN, TMC_UART_RX_PIN, TMC_SERIAL_BAUD);
    
    // Debug: Manual UART test - send read request for IOIN register (addr 0x06)
    // Datagram: sync+reserved(0x05), slave_addr(0x00), register(0x06), CRC(0x6F)
    Serial.println("StepperControl: Manual UART test to address 0...");
    _tmcSerial->flush();
    delay(10);
    
    // Clear any garbage
    while (_tmcSerial->available()) _tmcSerial->read();
    
    // Send read request for IOIN register
    uint8_t txData[] = {0x05, 0x00, 0x06, 0x6F};  // Read IOIN from address 0
    _tmcSerial->write(txData, 4);
    _tmcSerial->flush();
    
    delay(50);  // Wait for response
    
    int bytesAvailable = _tmcSerial->available();
    Serial.printf("StepperControl: Bytes received: %d\n", bytesAvailable);
    
    if (bytesAvailable > 0) {
        Serial.print("StepperControl: Response bytes: ");
        while (_tmcSerial->available()) {
            uint8_t b = _tmcSerial->read();
            Serial.printf("0x%02X ", b);
        }
        Serial.println();
    } else {
        Serial.println("StepperControl: No response - check wiring!");
        Serial.println("  - Is 1k resistor between ESP32 TX and TMC2209 RX?");
        Serial.println("  - Is ESP32 RX also connected to TMC2209 RX (after resistor)?");
        Serial.println("  - Is TMC2209 VIO powered (3.3V)?");
        Serial.println("  - Is TMC2209 VM powered (12-24V)?");
        Serial.println("  - Is TMC2209 EN pin LOW (GND)?");
    }
    
    // Create TMC2209 driver instances
    _tmcAz = new TMC2209Stepper(_tmcSerial, TMC_R_SENSE, TMC_AZ_ADDRESS);
    _tmcEl = new TMC2209Stepper(_tmcSerial, TMC_R_SENSE, TMC_EL_ADDRESS);
    
    // Small delay for drivers to power up
    delay(100);
    
    // Initialize both drivers (attempt UART configuration)
    initDriver(_tmcAz, TMC_AZ_ADDRESS);
    initDriver(_tmcEl, TMC_EL_ADDRESS);
    
    // Enable motors BEFORE checking UART - STEP/DIR works without UART
    // This ensures basic motor operation even if UART communication fails
    setEnabled(true);
    
    _lastUpdateMicros = micros();
    
    // Verify UART communication (warn but don't fail - motors still work via STEP/DIR)
    if (!driversConnected()) {
        DEBUG_PRINTLN("StepperControl: WARNING - UART communication failed!");
        DEBUG_PRINTLN("StepperControl: Motors enabled but running with default driver settings");
        DEBUG_PRINTLN("StepperControl: (Current control and StallGuard not available)");
        // Return true - basic STEP/DIR operation will still work
        return true;
    }
    
    DEBUG_PRINTLN("StepperControl: Initialization complete (UART OK)");
    return true;
}

void StepperControl::initDriver(TMC2209Stepper* driver, uint8_t address) {
    driver->begin();
    
    // Basic configuration
    driver->toff(5);                    // Enable driver (higher = more torque but more noise)
    driver->blank_time(24);             // Comparator blank time
    driver->rms_current(MOTOR_RUN_CURRENT_MA);
    driver->hold_multiplier(MOTOR_HOLD_MULTIPLIER);
    driver->microsteps(TMC_MICROSTEPS);
    driver->intpol(true);               // Interpolation to 256 microsteps
    
    // Use SpreadCycle for more torque (StealthChop is quieter but less torque)
    // SpreadCycle is better for geared steppers that need consistent torque
    driver->en_spreadCycle(true);       // SpreadCycle ON for torque
    driver->pwm_autoscale(true);        // Still enable PWM autoscale
    driver->pwm_autograd(true);         // Enable automatic gradient adaptation
    
    // TPWMTHRS: threshold for switching between StealthChop and SpreadCycle
    // With en_spreadCycle=true, we stay in SpreadCycle
    // Set to 0 to always use SpreadCycle (maximum torque)
    driver->TPWMTHRS(0);
    
    // Chopper configuration for better torque
    driver->hysteresis_start(4);        // Hysteresis start value
    driver->hysteresis_end(0);          // Hysteresis end value
    
    // StallGuard configuration
    driver->TCOOLTHRS(0xFFFFF);         // Enable StallGuard at all speeds
    driver->semin(5);                   // Minimum current
    driver->semax(2);                   // Maximum current increase
    driver->sedn(0b01);                 // Current down step speed
    driver->SGTHRS(TMC_STALL_THRESHOLD);
    
    DEBUG_PRINTF("StepperControl: Driver 0x%02X configured (SpreadCycle mode), version=0x%02X\n", 
                 address, driver->version());
}

void StepperControl::update() {
    if (!_enabled) return;
    
    // DESIGN NOTE on mutex timeout:
    // - update() is called from a real-time stepper task at high frequency
    // - Uses 1ms timeout to avoid blocking the stepper timing loop
    // - If mutex is unavailable (held by move commands), skip this update cycle
    // - This is acceptable because move commands complete quickly
    // - Non-time-critical functions (moveTo, stop) use portMAX_DELAY to ensure they
    //   always complete without missing updates
    if (xSemaphoreTake(_profileMutex, pdMS_TO_TICKS(1)) != pdTRUE) {
        return;  // Skip this update if mutex unavailable
    }
    
    uint32_t now = micros();
    
    // Update both axes
    if (_azProfile.isMoving) {
        updateAxis(_azProfile, TMC_AZ_STEP_PIN, TMC_AZ_DIR_PIN, 
                   _azLastStepMicros, AZ_MIN_DEG, AZ_MAX_DEG, AZ_MOTOR_REVERSED);
    }
    
    if (_elProfile.isMoving) {
        updateAxis(_elProfile, TMC_EL_STEP_PIN, TMC_EL_DIR_PIN,
                   _elLastStepMicros, EL_MIN_DEG, EL_MAX_DEG, EL_MOTOR_REVERSED);
    }
    
    xSemaphoreGive(_profileMutex);
    
    _lastUpdateMicros = now;
}

void StepperControl::updateAxis(MotionProfile& profile, uint8_t stepPin, uint8_t dirPin,
                                 uint32_t& lastStepMicros, float minPos, float maxPos,
                                 bool reversed) {
    if (!profile.isMoving) return;
    
    uint32_t now = micros();
    
    // Calculate step position difference
    int32_t stepsToGo = profile.targetSteps - profile.currentSteps;
    
    if (stepsToGo == 0) {
        // Reached target
        profile.isMoving = false;
        profile.currentSpeed = 0;
        profile.currentPosition = profile.targetPosition;
        return;
    }
    
    // Determine logical direction (toward target position)
    bool movingPositive = stepsToGo > 0;  // True = moving toward higher position value
    
    // Physical direction pin (apply reversal if configured)
    bool pinHigh = movingPositive;
    if (reversed) pinHigh = !pinHigh;  // Only invert the pin, not the step counting
    digitalWrite(dirPin, pinHigh ? HIGH : LOW);
    
    // Calculate distance to target (for deceleration)
    float distanceToTarget = abs(stepsToDegrees(stepsToGo));
    
    // Calculate stopping distance at current speed
    float stoppingDistance = (profile.currentSpeed * profile.currentSpeed) / 
                             (2.0f * profile.acceleration);
    
    // Determine target speed (accelerate or decelerate)
    float targetSpeed;
    if (distanceToTarget <= stoppingDistance) {
        // Need to decelerate
        targetSpeed = sqrt(2.0f * profile.acceleration * distanceToTarget);
    } else {
        // Can accelerate or maintain speed
        targetSpeed = profile.maxSpeed;
    }
    
    // Apply acceleration limit
    float dt = (now - lastStepMicros) / 1000000.0f;
    if (dt > STEPPER_DT_CAP_SEC) dt = STEPPER_DT_CAP_SEC;  // Cap dt to prevent large jumps
    
    if (profile.currentSpeed < targetSpeed) {
        profile.currentSpeed += profile.acceleration * dt;
        if (profile.currentSpeed > targetSpeed) {
            profile.currentSpeed = targetSpeed;
        }
    } else if (profile.currentSpeed > targetSpeed) {
        profile.currentSpeed -= profile.acceleration * dt;
        if (profile.currentSpeed < targetSpeed) {
            profile.currentSpeed = targetSpeed;
        }
    }
    
    // Minimum speed to prevent stalling
    if (profile.currentSpeed < STEPPER_MIN_SPEED_DEG_S) {
        profile.currentSpeed = STEPPER_MIN_SPEED_DEG_S;
    }
    
    // Only recalculate step interval when speed changes
    // This avoids FPU division every 1ms update cycle
    uint32_t stepIntervalMicros;
    if (profile.cachedSpeed != profile.currentSpeed) {
        float stepsPerSecond = profile.currentSpeed * STEPS_PER_DEGREE;
        stepIntervalMicros = (uint32_t)(1000000.0f / stepsPerSecond);
        
        // Minimum step interval (maximum frequency)
        if (stepIntervalMicros < STEPPER_MIN_STEP_INTERVAL_US) stepIntervalMicros = STEPPER_MIN_STEP_INTERVAL_US;
        
        // Cache the calculated values
        profile.cachedStepInterval = stepIntervalMicros;
        profile.cachedSpeed = profile.currentSpeed;
    } else {
        stepIntervalMicros = profile.cachedStepInterval;
    }
    
    // Time to step?
    if (now - lastStepMicros >= stepIntervalMicros) {
        // Generate step pulse
        pulseStep(stepPin);
        
        // Update position based on LOGICAL direction (not physical pin direction)
        if (movingPositive) {
            profile.currentSteps++;
        } else {
            profile.currentSteps--;
        }
        profile.currentPosition = stepsToDegrees(profile.currentSteps);
        
        lastStepMicros = now;
    }
}

void StepperControl::pulseStep(uint8_t stepPin) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(stepPin, LOW);
}

void StepperControl::moveAzimuthTo(float degrees, float speed) {
    if (!_enabled) return;
    
    // Apply offset
    float targetWithOffset = degrees + _azOffset;
    
    // Normalize and find shortest path
    float current = _azProfile.currentPosition;
    float target = shortestPath(current, targetWithOffset);
    
    // Clamp to limits
    if (target < AZ_MIN_DEG) target = AZ_MIN_DEG;
    if (target > AZ_MAX_DEG) target = AZ_MAX_DEG;
    
    // Protect motion profile access
    xSemaphoreTake(_profileMutex, portMAX_DELAY);
    
    _azProfile.targetPosition = target;
    _azProfile.targetSteps = degreesToSteps(target);
    _azProfile.maxSpeed = (speed > 0) ? speed : TRACKING_SPEED_DEG_S;
    
    // Start at minimum speed for smooth acceleration
    // Don't start at 0 - this causes timing issues
    _azProfile.currentSpeed = STEPPER_MIN_SPEED_DEG_S;
    _azProfile.cachedSpeed = -1.0f;  // Force recalculation of step interval
    
    _azProfile.isMoving = true;
    
    // Reset step timing for clean start
    _azLastStepMicros = micros();
    
    _azState = MotorState::MOVING;
    
    xSemaphoreGive(_profileMutex);
    
    DEBUG_PRINTF("StepperControl: Az move to %.2f° (speed %.1f°/s)\n", target, _azProfile.maxSpeed);
}

void StepperControl::moveElevationTo(float degrees, float speed) {
    if (!_enabled) return;
    
    // Apply offset
    float target = degrees + _elOffset;
    
    // Clamp to limits
    if (target < EL_MIN_DEG) target = EL_MIN_DEG;
    if (target > EL_MAX_DEG) target = EL_MAX_DEG;
    
    // Protect motion profile access
    xSemaphoreTake(_profileMutex, portMAX_DELAY);
    
    _elProfile.targetPosition = target;
    _elProfile.targetSteps = degreesToSteps(target);
    _elProfile.maxSpeed = (speed > 0) ? speed : TRACKING_SPEED_DEG_S;
    
    // Start at minimum speed for smooth acceleration
    _elProfile.currentSpeed = STEPPER_MIN_SPEED_DEG_S;
    _elProfile.cachedSpeed = -1.0f;  // Force recalculation of step interval
    
    _elProfile.isMoving = true;
    
    // Reset step timing for clean start
    _elLastStepMicros = micros();
    
    _elState = MotorState::MOVING;
    
    xSemaphoreGive(_profileMutex);
    
    DEBUG_PRINTF("StepperControl: El move to %.2f° (speed %.1f°/s)\n", target, _elProfile.maxSpeed);
}

void StepperControl::moveTo(float azDegrees, float elDegrees, float speed) {
    moveAzimuthTo(azDegrees, speed);
    moveElevationTo(elDegrees, speed);
}

void StepperControl::stop() {
    // Use RAII guard for mutex protection - ensures release even on early return
    MutexGuard guard(_profileMutex);
    if (!guard.acquired()) {
        DEBUG_PRINTLN("StepperControl: Failed to acquire mutex in stop()");
        return;
    }
    
    _azProfile.isMoving = false;
    _azProfile.currentSpeed = 0;
    _azProfile.targetPosition = _azProfile.currentPosition;
    _azProfile.targetSteps = _azProfile.currentSteps;
    
    _elProfile.isMoving = false;
    _elProfile.currentSpeed = 0;
    _elProfile.targetPosition = _elProfile.currentPosition;
    _elProfile.targetSteps = _elProfile.currentSteps;
    
    _azState = MotorState::IDLE;
    _elState = MotorState::IDLE;
    
    // Mutex automatically released by MutexGuard destructor
    DEBUG_PRINTLN("StepperControl: Motion stopped");
}

void StepperControl::emergencyStop() {
    stop();
    setEnabled(false);
    DEBUG_PRINTLN("StepperControl: EMERGENCY STOP");
}

void StepperControl::setEnabled(bool enabled) {
    _enabled = enabled;
    digitalWrite(TMC_EN_PIN, enabled ? LOW : HIGH);  // Active LOW
    
    if (!enabled) {
        _azProfile.isMoving = false;
        _elProfile.isMoving = false;
        _azState = MotorState::IDLE;
        _elState = MotorState::IDLE;
    }
    
    DEBUG_PRINTF("StepperControl: Motors %s\n", enabled ? "ENABLED" : "DISABLED");
}

bool StepperControl::home() {
    DEBUG_PRINTLN("StepperControl: Starting homing sequence...");
    
    // Home elevation first (down to 0°)
    if (!homeAxis(Axis::ELEVATION)) {
        DEBUG_PRINTLN("StepperControl: Elevation homing failed");
        return false;
    }
    
    // Then home azimuth
    if (!homeAxis(Axis::AZIMUTH)) {
        DEBUG_PRINTLN("StepperControl: Azimuth homing failed");
        return false;
    }
    
    DEBUG_PRINTLN("StepperControl: Homing complete");
    return true;
}

bool StepperControl::homeAxis(Axis axis) {
    TMC2209Stepper* driver = (axis == Axis::AZIMUTH) ? _tmcAz : _tmcEl;
    uint8_t stepPin = (axis == Axis::AZIMUTH) ? TMC_AZ_STEP_PIN : TMC_EL_STEP_PIN;
    uint8_t dirPin = (axis == Axis::AZIMUTH) ? TMC_AZ_DIR_PIN : TMC_EL_DIR_PIN;
    uint8_t diagPin = (axis == Axis::AZIMUTH) ? TMC_AZ_DIAG_PIN : TMC_EL_DIAG_PIN;
    MotionProfile& profile = (axis == Axis::AZIMUTH) ? _azProfile : _elProfile;
    MotorState& state = (axis == Axis::AZIMUTH) ? _azState : _elState;
    bool reversed = (axis == Axis::AZIMUTH) ? AZ_MOTOR_REVERSED : EL_MOTOR_REVERSED;
    
    const char* axisName = (axis == Axis::AZIMUTH) ? "Azimuth" : "Elevation";
    
    DEBUG_PRINTF("StepperControl: Homing %s...\n", axisName);
    
    state = MotorState::HOMING;
    
    // Configure for StallGuard detection
    driver->TCOOLTHRS(0xFFFFF);
    driver->SGTHRS(TMC_STALL_THRESHOLD);
    
    // Direction: elevation down (false), azimuth CCW (true)
    // Apply motor reversal if configured
    bool direction = (axis == Axis::ELEVATION) ? false : true;
    if (reversed) direction = !direction;
    digitalWrite(dirPin, direction ? HIGH : LOW);
    
    // Move until stall detected
    uint32_t stepInterval = (uint32_t)(1000000.0f / (HOMING_SPEED_DEG_S * STEPS_PER_DEGREE));
    uint32_t timeout = millis() + 30000;  // 30 second timeout
    uint32_t lastStep = micros();
    uint32_t yieldCounter = 0;
    
    while (millis() < timeout) {
        // Check for stall
        if (digitalRead(diagPin) == HIGH || driver->diag()) {
            DEBUG_PRINTF("StepperControl: %s stall detected!\n", axisName);
            vTaskDelay(pdMS_TO_TICKS(100));  // Non-blocking delay
            
            // Back off slightly
            digitalWrite(dirPin, !direction ? HIGH : LOW);
            for (int i = 0; i < 100; i++) {
                pulseStep(stepPin);
                delayMicroseconds(stepInterval);
                
                // Yield every 10 steps to avoid starving other tasks
                if (i % 10 == 0) {
                    vTaskDelay(1);  // Minimal delay to yield
                }
            }
            
            // Set position to zero
            profile.currentSteps = 0;
            profile.currentPosition = 0;
            profile.targetSteps = 0;
            profile.targetPosition = 0;
            profile.isMoving = false;
            profile.currentSpeed = 0;
            
            state = MotorState::IDLE;
            
            DEBUG_PRINTF("StepperControl: %s homed to 0°\n", axisName);
            return true;
        }
        
        // Step if interval elapsed
        if (micros() - lastStep >= stepInterval) {
            pulseStep(stepPin);
            lastStep = micros();
            yieldCounter++;
        }
        
        // Yield to other FreeRTOS tasks periodically (every ~10ms worth of steps)
        // This prevents starving other tasks during the 30-second timeout
        if (yieldCounter >= 100) {
            vTaskDelay(1);  // Yield to scheduler (1 tick minimum)
            yieldCounter = 0;
        }
    }
    
    DEBUG_PRINTF("StepperControl: %s homing timeout!\n", axisName);
    state = MotorState::ERROR;
    return false;
}

void StepperControl::setHome() {
    _azProfile.currentSteps = 0;
    _azProfile.currentPosition = 0;
    _azProfile.targetSteps = 0;
    _azProfile.targetPosition = 0;
    
    _elProfile.currentSteps = 0;
    _elProfile.currentPosition = 0;
    _elProfile.targetSteps = 0;
    _elProfile.targetPosition = 0;
    
    DEBUG_PRINTLN("StepperControl: Current position set as home (0,0)");
}

void StepperControl::park() {
    DEBUG_PRINTLN("StepperControl: Moving to park position");
    moveTo(PARK_AZ_DEG, PARK_EL_DEG, MAX_SLEW_SPEED_DEG_S);
}

float StepperControl::getAzimuth() const {
    return _azProfile.currentPosition - _azOffset;
}

float StepperControl::getElevation() const {
    return _elProfile.currentPosition - _elOffset;
}

bool StepperControl::isMoving() const {
    return _azProfile.isMoving || _elProfile.isMoving;
}

MotorState StepperControl::getState(Axis axis) const {
    return (axis == Axis::AZIMUTH) ? _azState : _elState;
}

void StepperControl::setCalibrationOffset(float azOffset, float elOffset) {
    _azOffset = azOffset;
    _elOffset = elOffset;
    DEBUG_PRINTF("StepperControl: Calibration offsets set: Az=%.2f°, El=%.2f°\n", 
                 azOffset, elOffset);
}

void StepperControl::setPosition(float azDegrees, float elDegrees) {
    _azProfile.currentPosition = azDegrees + _azOffset;
    _azProfile.currentSteps = degreesToSteps(_azProfile.currentPosition);
    _azProfile.targetPosition = _azProfile.currentPosition;
    _azProfile.targetSteps = _azProfile.currentSteps;
    
    _elProfile.currentPosition = elDegrees + _elOffset;
    _elProfile.currentSteps = degreesToSteps(_elProfile.currentPosition);
    _elProfile.targetPosition = _elProfile.currentPosition;
    _elProfile.targetSteps = _elProfile.currentSteps;
    
    DEBUG_PRINTF("StepperControl: Position set to Az=%.2f°, El=%.2f°\n", 
                 azDegrees, elDegrees);
}

void StepperControl::setMotorCurrent(uint16_t runCurrent_mA, uint16_t holdCurrent_mA) {
    _tmcAz->rms_current(runCurrent_mA);
    _tmcEl->rms_current(runCurrent_mA);
    
    float holdMult = (float)holdCurrent_mA / (float)runCurrent_mA;
    _tmcAz->hold_multiplier(holdMult);
    _tmcEl->hold_multiplier(holdMult);
    
    DEBUG_PRINTF("StepperControl: Motor current set: run=%dmA, hold=%dmA\n",
                 runCurrent_mA, holdCurrent_mA);
}

uint32_t StepperControl::getDriverStatus(Axis axis) {
    TMC2209Stepper* driver = (axis == Axis::AZIMUTH) ? _tmcAz : _tmcEl;
    return driver->DRV_STATUS();
}

bool StepperControl::driversConnected() {
    // Skip UART test during motion - causes pauses!
    if (isMoving()) {
        return true;  // Assume OK during motion
    }
    
    bool azOk = (_tmcAz->test_connection() == 0);
    bool elOk = (_tmcEl->test_connection() == 0);
    
    // Only print on failure (avoid spam during normal operation)
    if (!azOk || !elOk) {
        DEBUG_PRINTF("StepperControl: Driver connection test - Az:%s, El:%s\n",
                     azOk ? "OK" : "FAIL", elOk ? "OK" : "FAIL");
    }
    
    return azOk && elOk;
}

float StepperControl::stepsToDegrees(int32_t steps) {
    return (float)steps / STEPS_PER_DEGREE;
}

int32_t StepperControl::degreesToSteps(float degrees) {
    return (int32_t)(degrees * STEPS_PER_DEGREE);
}

bool StepperControl::detectStall(Axis axis) {
    uint8_t diagPin = (axis == Axis::AZIMUTH) ? TMC_AZ_DIAG_PIN : TMC_EL_DIAG_PIN;
    TMC2209Stepper* driver = (axis == Axis::AZIMUTH) ? _tmcAz : _tmcEl;
    
    return digitalRead(diagPin) == HIGH || driver->diag();
}

float StepperControl::normalizeAzimuth(float degrees) {
    // Use fmod for single operation instead of while loops
    // fmod can return negative values, so we need to handle that
    float normalized = fmodf(degrees, 360.0f);
    if (normalized < 0) {
        normalized += 360.0f;
    }
    return normalized;
}

float StepperControl::shortestPath(float current, float target) {
    // Calculate the shortest path considering wrap-around
    float normalized_target = normalizeAzimuth(target);
    float normalized_current = normalizeAzimuth(current);
    
    float diff = normalized_target - normalized_current;
    
    // Find shortest path
    if (diff > 180.0f) {
        return current + (diff - 360.0f);
    } else if (diff < -180.0f) {
        return current + (diff + 360.0f);
    } else {
        return current + diff;
    }
}
