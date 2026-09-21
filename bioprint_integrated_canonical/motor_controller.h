/*
 * BioPrintAM Motor Controller Library
 * ===================================
 * Unified motor control and TIC stepper driver management for all BioPrintAM firmware variants.
 * 
 * Features:
 * - TIC I2C driver initialization and configuration
 * - Synchronized dual-motor movement with flexible speed control
 * - Robust position tracking with error handling
 * - Conversion utilities (mL → steps, mm/s ↔ step/sec)
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
 *   MotorMoveState move;
 *   void loop() {
 *     // Non-blocking: start once, then poll once per loop() pass
 *     // startMotorMove(move, 5000, 5000, 10.0, 10.0);  // targets in steps, speeds in mm/s
 *     // MotorMoveStatus s = pollMotorMove(move);       // MOVING / ARRIVED / FAILED
 *   }
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Tic.h>
#include <Wire.h>
#include "config.h"

const int MAX_STATIONARY = 20;

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
 * Convert a syringe volume to the plunger position that holds it
 * Formula: position = ZERO_ML_POSITION + volume_ml * STEPS_PER_ML
 */
inline long volumeToPosition(float ml) {
  return ZERO_ML_POSITION + mlToSteps(ml);
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

enum MotorMoveStatus { MOVING, ARRIVED, FAILED};

struct MotorMoveState {
  long target1, target2;
  unsigned long start_time;
  long last_pos1, last_pos2;
  int stationary_count;
  int consecutive_arrivals;
};

// A move (one or two phases) that loop() services one poll per pass.
// Always start one with arm() (plus addPhase() for a second phase): they set
// every field loop() reads, so nothing stale from a previous move can leak in.
// On failure loop() halts the motors, logs failureMessage and shows the error page.
struct PendingMove {
  MotorMoveState moveState;
  bool active = false;
  bool phaseStarted = false;
  uint8_t phaseCount, phaseIndex;
  long target1_phase1, target2_phase1;
  long target1_phase2, target2_phase2;
  float speed1_phase1, speed2_phase1;
  float speed1_phase2, speed2_phase2;
  void (*onArrived)();
  String failureMessage;
  void (*onProgress)();

  // Arm a single-phase move to (t1, t2) at `speed` mm/s for both motors.
  // onProgress is called every pass while moving, onArrived once at the end.
  void arm(long t1, long t2, float speed,
           void (*progress)(), void (*arrived)(), const char* failMsg) {
    target1_phase1 = t1;
    target2_phase1 = t2;
    speed1_phase1 = speed;
    speed2_phase1 = speed;
    phaseCount = 1;
    phaseIndex = 0;
    phaseStarted = false;
    onProgress = progress;
    onArrived = arrived;
    failureMessage = failMsg;
    active = true;   // last: loop() services the move as soon as this is set
  }

  // Chain a second phase after arm(); call it right after, before returning to loop().
  void addPhase(long t1, long t2, float speed) {
    target1_phase2 = t1;
    target2_phase2 = t2;
    speed1_phase2 = speed;
    speed2_phase2 = speed;
    phaseCount = 2;
  }
};

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

// ==================== BASIC MOVEMENT FUNCTIONS ====================


inline void startMotorMove(MotorMoveState &state, long target1, long target2, float speed_mms1, float speed_mms2) {
  syncPositionToTIC();
  tic1.clearDriverError();
  tic2.clearDriverError();
  tic1.exitSafeStart();
  tic2.exitSafeStart();

  tic1.setMaxSpeed(stepsPerSecToTicUnits(mmsToStepsPerSec(speed_mms1)));
  tic2.setMaxSpeed(stepsPerSecToTicUnits(mmsToStepsPerSec(speed_mms2)));
  tic1.setTargetPosition(target1);
  tic2.setTargetPosition(target2);

  state.target1 = target1;
  state.target2 = target2;
  state.start_time = millis();
  state.last_pos1 = arduino_pos1;
  state.last_pos2 = arduino_pos2;
  state.stationary_count = 0;
  state.consecutive_arrivals = 0;
}

inline MotorMoveStatus pollMotorMove(MotorMoveState &state){
  tic1.resetCommandTimeout();
  tic2.resetCommandTimeout();

  long pos1 = tic1.getCurrentPosition();
  long pos2 = tic2.getCurrentPosition();
  uint16_t err1 = tic1.getErrorStatus();
  uint16_t err2 = tic2.getErrorStatus();

  if (err1 != 0 || err2 != 0) {
    arduino_pos1 = pos1;
    arduino_pos2 = pos2;
    return FAILED;
  }

  if (pos1 == state.last_pos1 && pos2 == state.last_pos2){
    state.stationary_count++;
    if (state.stationary_count >= MAX_STATIONARY) {
      bool at_target1 = abs(pos1 - state.target1) <= POSITION_TOLERANCE;
      bool at_target2 = abs(pos2 - state.target2) <= POSITION_TOLERANCE;
      if (at_target1 && at_target2) {
        arduino_pos1 = state.target1;
        arduino_pos2 = state.target2;
        return ARRIVED;
      } else {
        arduino_pos1 = pos1;
        arduino_pos2 = pos2;
        return FAILED;
      }
    }
  } else {
    state.stationary_count = 0;
    state.last_pos1 = pos1;
    state.last_pos2 = pos2;
  }

  bool arrived1 = abs(pos1 - state.target1) <= POSITION_TOLERANCE;
  bool arrived2 = abs(pos2 - state.target2) <= POSITION_TOLERANCE;
  
  if (arrived1 && arrived2) {
    state.consecutive_arrivals++;
    if (state.consecutive_arrivals >= 3) {
      arduino_pos1 = state.target1;
      arduino_pos2 = state.target2;
      return ARRIVED;
    }
  } else {
    state.consecutive_arrivals = 0;
  }

  if (millis() - state.start_time > 60000) {
    arduino_pos1 = pos1;
    arduino_pos2 = pos2;
    return FAILED;
  }
  return MOVING;
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
