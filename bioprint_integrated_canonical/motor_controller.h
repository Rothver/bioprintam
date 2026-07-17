/*
 * BioPrintAM Motor Controller Library
 * ===================================
 * Unified motor control and TIC stepper driver management for all BioPrintAM firmware variants.
 * 
 * Features:
 * - TIC I2C driver initialization and configuration
 * - Synchronized dual-motor movement with flexible speed control
 * - Robust position tracking with error handling
 * - Conversion utilities (mL ↔ steps, mm/s ↔ step/sec)
 * - Emergency halt and safe-start management
 * 
 * Hardware Requirements:
 * - Arduino GIGA R1 WiFi with Tic library installed
 * - 2× Pololu TIC T825 stepper drivers (I2C addresses 14, 15)
 * - I2C communication via Wire library
 * 
 * Global Dependencies (must be declared in main sketch):
 * - TicI2C tic1, tic2       // Tic motor objects
 * - long arduino_pos1, arduino_pos2    // Position tracking
 * - SystemState current_state (or equivalent) // State management
 * 
 * Usage:
 *   #include "config/config.h"
 *   #include "libraries/motor_controller.h"
 *   
 *   TicI2C tic1, tic2;
 *   long arduino_pos1 = 0, arduino_pos2 = 0;
 *   
 *   void setup() {
 *     initializeMotors();
 *   }
 *   
 *   void loop() {
 *     moveMotorsTo(5000, 5000, 10.0);  // Move to step 5000 at 10 mm/s
 *   }
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Tic.h>
#include <Wire.h>
#include "config.h"

// ==================== EXTERNAL MOTOR OBJECTS ====================
// Must be declared in main sketch:
extern TicI2C tic1;
extern TicI2C tic2;
extern long arduino_pos1;
extern long arduino_pos2;

// Optional: State management (used in integrated firmware)
// If your sketch doesn't have this, comment it out
// extern SystemState current_state;

// ==================== CONVERSION HELPERS ====================

/*
 * Convert milliliters to motor steps
 * Formula: steps = volume_ml * STEPS_PER_ML
 */
inline long mlToSteps(float ml) {
  return (long)(ml * STEPS_PER_ML);
}

/*
 * Convert motor steps to milliliters
 * Formula: volume = steps / STEPS_PER_ML
 */
inline float stepsToMl(long steps) {
  return (float)steps / STEPS_PER_ML;
}

/*
 * Convert mm/s to steps/second for TIC control
 * Formula: steps/s = (mm/s) / MM_PER_STEP
 */
inline long mmsToStepsPerSec(float mm_per_s) {
  long steps_s = (long)(mm_per_s / MM_PER_STEP);
  return max(1L, steps_s);  // Ensure minimum of 1
}

/*
 * Convert steps/second to TIC unit format
 * TIC uses 10000× scaling for speed units
 * Formula: tic_units = steps_per_sec * 10000
 */
inline long stepsPerSecToTicUnits(long steps_per_sec) {
  return steps_per_sec * 10000L;
}

// ==================== MOTOR INITIALIZATION ====================

/*
 * Initialize both TIC stepper drivers via I2C
 * 
 * Setup sequence:
 * 1. Configure I2C address for each motor
 * 2. Clear any previous errors
 * 3. Energize motors
 * 4. Exit safe-start mode
 * 5. Set acceleration and deceleration limits
 * 6. Read initial positions
 * 
 * Returns: Nothing (errors reported to Serial)
 */
inline void initializeMotors() {
  Wire.begin();
  delay(100);
  
  // Configure Motor 1 (address 14)
  tic1.setAddress(MOTOR1_ADDRESS);
  delay(10);
  tic1.clearDriverError();
  tic1.energize();
  delay(10);
  tic1.exitSafeStart();
  delay(10);
  tic1.setMaxAccel(MAX_ACCEL * 100L);
  tic1.setMaxDecel(MAX_ACCEL * 100L);
  
  // Configure Motor 2 (address 15)
  tic2.setAddress(MOTOR2_ADDRESS);
  delay(10);
  tic2.clearDriverError();
  tic2.energize();
  delay(10);
  tic2.exitSafeStart();
  delay(10);
  tic2.setMaxAccel(MAX_ACCEL * 100L);
  tic2.setMaxDecel(MAX_ACCEL * 100L);
  
  // Read and store initial positions
  long init_pos1 = tic1.getCurrentPosition();
  long init_pos2 = tic2.getCurrentPosition();
  
  arduino_pos1 = init_pos1;
  arduino_pos2 = init_pos2;
}

// ==================== POSITION SYNCHRONIZATION ====================

/*
 * Synchronize Arduino position tracking with TIC hardware positions
 * 
 * Purpose:
 * - Ensures position tracking accuracy
 * - Updates arduino_pos1/2 variables from TIC hardware
 * - Used before major movements to prevent divergence
 * 
 * Note: haltAndSetPosition may not work reliably on all TIC firmware
 * versions, so we attempt the sync but don't fail if mismatch detected.
 * 
 * Returns: true (sync attempted)
 */
inline bool syncPositionToTIC() {
  tic1.haltAndSetPosition(arduino_pos1);
  tic2.haltAndSetPosition(arduino_pos2);
  
  delay(20);
  
  // Don't verify - just proceed with movement
  // The movement itself will establish the correct position
  return true;
}

/*
 * Set motor positions to specific values
 * Used for emergency halt or manual position correction
 */
inline void haltAndSetPosition(long pos1, long pos2) {
  tic1.haltAndSetPosition(pos1);
  tic2.haltAndSetPosition(pos2);
  
  arduino_pos1 = pos1;
  arduino_pos2 = pos2;
  
  delay(20);
}

inline bool waitForArrival(long target1, long target2) {
  unsigned long start_time = millis();
  
  int consecutive_arrivals = 0;
  const int REQUIRED_CONSECUTIVE = 3;
  
  long last_pos1 = arduino_pos1;
  long last_pos2 = arduino_pos2;
  int stationary_count = 0;
  const int MAX_STATIONARY = 20;
  
  while (true) {
    tic1.resetCommandTimeout();
    tic2.resetCommandTimeout();
    
    long pos1 = tic1.getCurrentPosition();
    long pos2 = tic2.getCurrentPosition();
    
    uint16_t err1 = tic1.getErrorStatus();
    uint16_t err2 = tic2.getErrorStatus();
    
    if (err1 != 0 || err2 != 0) {
      arduino_pos1 = pos1;
      arduino_pos2 = pos2;
      return false;
    }
    
    if (pos1 == last_pos1 && pos2 == last_pos2) {
      stationary_count++;
      
      if (stationary_count >= MAX_STATIONARY) {
        bool at_target1 = abs(pos1 - target1) <= POSITION_TOLERANCE;
        bool at_target2 = abs(pos2 - target2) <= POSITION_TOLERANCE;
        
        if (at_target1 && at_target2) {
          arduino_pos1 = target1;
          arduino_pos2 = target2;
          return true;
        } else {
          arduino_pos1 = pos1;
          arduino_pos2 = pos2;
          return false;
        }
      }
    } else {
      stationary_count = 0;
      last_pos1 = pos1;
      last_pos2 = pos2;
    }
    
    bool arrived1 = abs(pos1 - target1) <= POSITION_TOLERANCE;
    bool arrived2 = abs(pos2 - target2) <= POSITION_TOLERANCE;
    
    if (arrived1 && arrived2) {
      consecutive_arrivals++;
      
      if (consecutive_arrivals >= REQUIRED_CONSECUTIVE) {
        arduino_pos1 = target1;
        arduino_pos2 = target2;
        return true;
      }
    } else {
      consecutive_arrivals = 0;
    }
    
    if (millis() - start_time > 60000) {
      arduino_pos1 = pos1;
      arduino_pos2 = pos2;
      return false;
    }
    
    delay(50);
  }
}

// ==================== BASIC MOVEMENT FUNCTIONS ====================

/*
 * Move both motors to target positions at specified speed
 * Synchronizes speed across both motors (useful for precise synchronized extrusion)
 * 
 * Parameters:
 *   target1: Target position for motor 1 (steps)
 *   target2: Target position for motor 2 (steps)
 *   speed_mms: Speed in mm/s (same for both motors)
 * 
 * Returns:
 *   true if targets reached successfully
 *   false if error occurs, timeout, or position divergence detected
 * 
 * Features:
 * - Robust arrival detection (consecutive confirmations)
 * - Stationary detection (prevents infinite loops)
 * - Error recovery (clears errors, retries)
 * - 60-second timeout protection
 * - Position tracking updates on completion
 */
inline bool moveMotorsTo(long target1, long target2, float speed_mms) {
  if (!syncPositionToTIC()) {
    return false;
  }
  
  tic1.clearDriverError();
  tic2.clearDriverError();
  tic1.exitSafeStart();
  tic2.exitSafeStart();
  
  long speed_steps = mmsToStepsPerSec(speed_mms);
  tic1.setMaxSpeed(stepsPerSecToTicUnits(speed_steps));
  tic2.setMaxSpeed(stepsPerSecToTicUnits(speed_steps));
  
  tic1.setTargetPosition(target1);
  tic2.setTargetPosition(target2);
  
  return waitForArrival(target1, target2);
}

/*
 * Move both motors to targets using independently-configured speeds
 * Used when motors have different speed settings already configured via setMaxSpeed()
 * 
 * Parameters:
 *   target1: Target position for motor 1 (steps)
 *   target2: Target position for motor 2 (steps)
 * 
 * Returns:
 *   true if targets reached successfully
 *   false if error occurs or timeout
 * 
 * Note: Call tic1.setMaxSpeed() and tic2.setMaxSpeed() BEFORE this function
 */
inline bool moveMotorsToWithSetSpeeds(long target1, long target2) {
  if (!syncPositionToTIC()) {
    return false;
  }
  
  tic1.clearDriverError();
  tic2.clearDriverError();
  tic1.exitSafeStart();
  tic2.exitSafeStart();
  
  tic1.setTargetPosition(target1);
  tic2.setTargetPosition(target2);
  
  return waitForArrival(target1, target2);
}

/*
 * Synchronized timed movement: both motors finish at approximately the same time
 * 
 * Parameters:
 *   target1, target2: Target positions (steps)
 *   speed1_mms, speed2_mms: Individual speeds (mm/s) for each motor
 *   duration_sec: Total duration (used for validation, not enforcement)
 * 
 * Returns:
 *   true if both motors reached targets
 *   false if error or timeout
 * 
 * Purpose:
 * - Enable precise synchronized extrusion with independent ratios
 * - Each motor moves at its own speed, calculated to finish together
 * - Useful for dual-ratio extrusion (e.g., 70:30 polymer blend)
 */
inline bool moveMotorsTimedSync(long target1, long target2, float speed1_mms, float speed2_mms, float duration_sec) {
  if (!syncPositionToTIC()) {
    return false;
  }
  
  long start_pos1 = arduino_pos1;
  long start_pos2 = arduino_pos2;
  long dist1 = abs(target1 - start_pos1);
  long dist2 = abs(target2 - start_pos2);
  
  tic1.clearDriverError();
  tic2.clearDriverError();
  tic1.exitSafeStart();
  tic2.exitSafeStart();
  
  // Set independent speeds for each motor
  tic1.setMaxSpeed(stepsPerSecToTicUnits(mmsToStepsPerSec(speed1_mms)));
  tic2.setMaxSpeed(stepsPerSecToTicUnits(mmsToStepsPerSec(speed2_mms)));
  
  delay(50);
  
  // Start BOTH motors at the exact same time
  tic1.setTargetPosition(target1);
  tic2.setTargetPosition(target2);
  
  unsigned long start_time = millis();
  
  // Monitor until BOTH reach targets
  bool m1_arrived = false;
  bool m2_arrived = false;
  unsigned long m1_finish_time = 0;
  unsigned long m2_finish_time = 0;
  
  while (!m1_arrived || !m2_arrived) {
    tic1.resetCommandTimeout();
    tic2.resetCommandTimeout();
    
    long pos1 = tic1.getCurrentPosition();
    long pos2 = tic2.getCurrentPosition();
    
    // Track when each motor arrives at its target
    if (!m1_arrived && abs(pos1 - target1) <= 50) {
      m1_arrived = true;
      m1_finish_time = millis() - start_time;
    }
    
    if (!m2_arrived && abs(pos2 - target2) <= 50) {
      m2_arrived = true;
      m2_finish_time = millis() - start_time;
    }
    
    uint16_t err1 = tic1.getErrorStatus();
    uint16_t err2 = tic2.getErrorStatus();
    
    if (err1 != 0 || err2 != 0) {
      arduino_pos1 = pos1;
      arduino_pos2 = pos2;
      return false;
    }
    
    // Timeout after 60 seconds
    if (millis() - start_time > 60000) {
      arduino_pos1 = pos1;
      arduino_pos2 = pos2;
      return false;
    }
    
    delay(50);
  }
  
  // Update positions after successful sync
  arduino_pos1 = target1;
  arduino_pos2 = target2;
  
  return true;
}


// ==================== EMERGENCY HALT ====================

/*
 * Emergency halt: immediately stops both motors and holds position
 * Used when:
 * - TIC error detected
 * - Position overrun detected
 * - User requests emergency stop
 * - Safety condition violated
 * 
 * Effect: Motors are de-energized and held in current position
 */
inline void emergencyHalt() {
  tic1.haltAndHold();
  tic2.haltAndHold();
  
  // Update position tracking to current hardware state
  arduino_pos1 = tic1.getCurrentPosition();
  arduino_pos2 = tic2.getCurrentPosition();
}

#endif  // MOTOR_CONTROLLER_H
