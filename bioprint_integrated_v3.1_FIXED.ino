/*
 * BioPrint AM - Integrated Control System v3.1 FIXED
 * Arduino GIGA R1 WiFi
 * 
 * Integration of:
 * - Improved motor control from syringe_controller_fixed_v2.ino
 * - Complete UI system from BioPrint AM
 * - PID temperature control
 * - Synchronized dual-syringe extrusion
 * 
 * Features:
 * - 15000 step LOAD position (0mL~1600, 10mL~11600)
 * - Synchronized speed control for different concentration ratios
 * - Infinite loop fixes (consecutive arrival + stationary detection)
 * - Full UI with touch control
 * - PID P-only control (Kp=150) for temperature
 * 
 * FIX: Proper coordinate system using 15000 as LOAD_POSITION
 *      Position 1600 = 0mL (testing apparatus offset)
 * 
 * Hardware:
 * - Arduino GIGA R1 WiFi
 * - 2× 10K NTC thermistors (β=3435) on A0, A1 (heat mats)
 * - 2× 10K NTC thermistors (β=3950) on A2, A3 (system monitoring)
 * - MOSFET on Pin 9 (PWM heat control)
 * - 2× Tic stepper controllers (I2C addresses 14, 15)
 */

#include <Tic.h>
#include <Wire.h>
#include <math.h>
#include "Arduino_GigaDisplay_GFX.h"
#include "Arduino_GigaDisplayTouch.h"
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include "config/config.h"
#include "libraries/thermistor_sensor.h"
#include "libraries/motor_controller.h"
#include "libraries/pid_controller.h"
#include "libraries/ui_system.h"
#include "libraries/state_machine.h"
#include "libraries/extrusion_helpers.h"

// ==================== DISPLAY & TOUCH ====================
GigaDisplay_GFX display;
Arduino_GigaDisplayTouch touchDetector;

// ==================== MOTOR CONFIGURATION ====================
#define MOTOR1_ADDRESS 14
#define MOTOR2_ADDRESS 15

// Motor constants (from syringe_controller_fixed_v2.ino)
const float STEPS_PER_ML = 1000.0f;
const float MM_PER_STEP = 0.00605f;
const float MM_PER_ML = 6.05f;
const float MAX_VOLUME_ML = 12.0f;
const long LOAD_POSITION = 15000L;  // Loading position at top
const float MAX_SPEED_MMS = 3.0f;
const long MAX_SPEED_STEPS_S = 496L;
const int MAX_ACCEL = 1000;
const int POSITION_TOLERANCE = 10;

TicI2C tic1;
TicI2C tic2;

// ==================== HEAT CONTROL CONFIGURATION ====================
const int MOSFET_PIN = 9;

// ==================== THERMISTOR CONFIGURATION ====================
// All thermistor calibration moved to config/config.h
// Sensor types defined in libraries/thermistor_sensor.h
// Temperature control state moved to libraries/pid_controller.h

// ==================== STATE MACHINE ====================
// All state and page enums moved to libraries/state_machine.h
SystemState current_state = UNINITIALIZED;
Page currentPage = WELCOME;

// System state tracking (temperature stabilization is now in pid_controller.h)
bool systemReady = false;
unsigned long tempStableTime = 0;

// ==================== POSITION TRACKING ====================
long arduino_pos1 = 0;
long arduino_pos2 = 0;
bool motorsHomed = false;

// ==================== UI PARAMETER STORAGE ====================
float selectedTemp = -1;
float selectedVol1 = -1;
float selectedVol2 = -1;
float selectedConc = -1;
float selectedSpeed = -1;
float tempSelection = -1;

// Extrusion setup variables
float extrusionVolume = 1.0;  // Total volume to extrude per cycle
float printTime = 5.0;        // Time in seconds to complete extrusion (2-15)

// Cycle tracking for progress
float cycleStartDispensed1 = 0.0;  // Dispensed amount at start of current cycle
float cycleStartDispensed2 = 0.0;
float cycleTargetVol1 = 0.0;       // Target volume for this cycle
float cycleTargetVol2 = 0.0;

// Zero retraction tracking
unsigned long retractionStartTime = 0;
const unsigned long RETRACTION_DURATION = 15000;  // 15 seconds in milliseconds
bool calibrationComplete = false;

// Shutdown tracking
unsigned long shutdownStartTime = 0;
bool shutdownInProgress = false;


// ==================== SYSTEM CONFIGURATION ====================
// SystemConfig struct moved to libraries/state_machine.h
SystemConfig config;

// ExtrusionValidation struct moved to libraries/state_machine.h

// UI options moved to libraries/state_machine.h
// Access via: TEMP_OPTIONS[], NUM_TEMP_OPTIONS, etc.

bool lastTouchState = false;
bool isPrinting = false;

// ==================== UTILITY FUNCTIONS ====================
// Motor conversion helpers now delegated to libraries/motor_controller.h
// See motor_controller.h for: mlToSteps, stepsToMl, mmsToStepsPerSec, stepsPerSecToTicUnits

// ==================== TEMPERATURE CONTROL FUNCTIONS (from pid_controller.h) ====================
// updateTemperatures(), computeDualPID(), applyHeatControl() are now defined in libraries/pid_controller.h
// These are called from the main loop and UI handlers without modification

// ==================== POSITION SYNCHRONIZATION ====================
bool syncPositionToTIC() {
  // Note: haltAndSetPosition may not work reliably on all TIC firmware
  // We'll attempt it but won't fail if verification doesn't match
  tic1.haltAndSetPosition(arduino_pos1);
  tic2.haltAndSetPosition(arduino_pos2);
  
  delay(20);
  
  // Don't verify - just proceed with movement
  // The movement itself will establish the correct position
  return true;
}

// ==================== MOTOR CONTROL (from syringe_controller_fixed_v2.ino) ====================
void initializeMotors() {
  Wire.begin();
  delay(100);
  
  tic1.setAddress(MOTOR1_ADDRESS);
  delay(10);
  tic1.clearDriverError();
  tic1.energize();
  delay(10);
  tic1.exitSafeStart();
  delay(10);
  tic1.setMaxAccel(MAX_ACCEL * 100L);
  tic1.setMaxDecel(MAX_ACCEL * 100L);
  
  tic2.setAddress(MOTOR2_ADDRESS);
  delay(10);
  tic2.clearDriverError();
  tic2.energize();
  delay(10);
  tic2.exitSafeStart();
  delay(10);
  tic2.setMaxAccel(MAX_ACCEL * 100L);
  tic2.setMaxDecel(MAX_ACCEL * 100L);
  
  long init_pos1 = tic1.getCurrentPosition();
  long init_pos2 = tic2.getCurrentPosition();
  
  arduino_pos1 = init_pos1;
  arduino_pos2 = init_pos2;
}

void calibrateMotorsOnStartup() {
  Serial.println("\n=== STARTUP CALIBRATION ===");
  Serial.println("Establishing MIN/MAX reference range (0-15000)");
  
  // Step 1: Move to MIN position (0)
  Serial.println("Step 1: Moving to MIN position (0)");
  if (!moveMotorsTo(0, 0, 3.0)) {
    Serial.println("ERROR: Failed to reach MIN position!");
    return;
  }
  delay(200);
  Serial.println("MIN position (0) reached");
  
  // Step 2: Move to MAX position (15000) - LOAD_POSITION
  Serial.println("Step 2: Moving to MAX position (15000)");
  if (!moveMotorsTo(LOAD_POSITION, LOAD_POSITION, 3.0)) {
    Serial.println("ERROR: Failed to reach MAX position!");
    return;
  }
  delay(200);
  
  arduino_pos1 = LOAD_POSITION;
  arduino_pos2 = LOAD_POSITION;
  
  Serial.println("=== CALIBRATION COMPLETE ===");
  Serial.print("Motors at LOAD_POSITION (");
  Serial.print(LOAD_POSITION);
  Serial.println(")");
  Serial.println("MIN=0, MAX=15000, 0mL=1600");
  Serial.println("Ready for syringe loading\n");
}

bool moveMotorsTo(long target1, long target2, float speed_mms) {
  if (!syncPositionToTIC()) {
    current_state = SAFE_MODE;
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
      current_state = SAFE_MODE;
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
          current_state = SAFE_MODE;
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
      current_state = SAFE_MODE;
      return false;
    }
    
    delay(50);
  }
}

bool moveMotorsToWithSetSpeeds(long target1, long target2) {
  if (!syncPositionToTIC()) {
    current_state = SAFE_MODE;
    return false;
  }
  
  tic1.clearDriverError();
  tic2.clearDriverError();
  tic1.exitSafeStart();
  tic2.exitSafeStart();
  
  tic1.setTargetPosition(target1);
  tic2.setTargetPosition(target2);
  
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
      current_state = SAFE_MODE;
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
          current_state = SAFE_MODE;
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
      current_state = SAFE_MODE;
      return false;
    }
    
    delay(50);
  }
}

// ==================== HOMING FUNCTIONS ====================
bool homeMotors() {
  // Read actual TIC positions
  long actual_pos1 = tic1.getCurrentPosition();
  long actual_pos2 = tic2.getCurrentPosition();
  
  Serial.print("homeMotors: Current positions M1=");
  Serial.print(actual_pos1);
  Serial.print(", M2=");
  Serial.println(actual_pos2);
  
  // Update arduino tracker
  arduino_pos1 = actual_pos1;
  arduino_pos2 = actual_pos2;
  
  // Check if already at LOAD_POSITION (15000)
  if (abs(actual_pos1 - LOAD_POSITION) <= POSITION_TOLERANCE && 
      abs(actual_pos2 - LOAD_POSITION) <= POSITION_TOLERANCE) {
    Serial.println("homeMotors: Already at LOAD_POSITION (15000)");
    arduino_pos1 = LOAD_POSITION;
    arduino_pos2 = LOAD_POSITION;
    Serial.println("homeMotors: SUCCESS (already there)");
    return true;
  }
  
  // Move to LOAD_POSITION (15000)
  Serial.println("homeMotors: Moving to LOAD_POSITION (15000)...");
  if (!moveMotorsTo(LOAD_POSITION, LOAD_POSITION, MAX_SPEED_MMS)) {
    Serial.println("homeMotors: FAILED to move");
    return false;
  }
  
  Serial.println("homeMotors: SUCCESS (moved to 15000)");
  return true;
}

bool completeHoming() {
  // Motors are already at LOAD_POSITION (15000) from startup calibration
  // Just move directly to volume position
  // Position system: 15000=load, 1600=0mL, 2600=1mL, 3600=2mL, etc.
  
  const long ZERO_ML_POSITION = 1600L;  // 0mL mark with current apparatus offset
  
  long target_pos1, target_pos2;
  
  Serial.println("=== PRIMING CALCULATION ===");
  Serial.print("Selected Vol1: ");
  Serial.print(selectedVol1, 2);
  Serial.print(" mL, Vol2: ");
  Serial.print(selectedVol2, 2);
  Serial.println(" mL");
  
  if (selectedVol1 > 0 && selectedVol2 > 0) {
    // Calculate target position: 0mL position + volume_steps
    long steps1 = mlToSteps(selectedVol1);
    long steps2 = mlToSteps(selectedVol2);
    
    Serial.print("Steps for Vol1: ");
    Serial.print(steps1);
    Serial.print(", Vol2: ");
    Serial.println(steps2);
    
    target_pos1 = ZERO_ML_POSITION + steps1;
    target_pos2 = ZERO_ML_POSITION + steps2;
    
    Serial.print("Target positions: M1=");
    Serial.print(target_pos1);
    Serial.print(" (");
    Serial.print(selectedVol1, 1);
    Serial.print("mL), M2=");
    Serial.print(target_pos2);
    Serial.print(" (");
    Serial.print(selectedVol2, 1);
    Serial.println("mL)");
  } else {
    // Default to 10mL (position 11600)
    target_pos1 = ZERO_ML_POSITION + mlToSteps(10.0);
    target_pos2 = ZERO_ML_POSITION + mlToSteps(10.0);
    
    Serial.print("Using default 10mL -> pos ");
    Serial.println(target_pos1);
  }
  
  Serial.print("Moving from LOAD (");
  Serial.print(LOAD_POSITION);
  Serial.print(") to M1=");
  Serial.print(target_pos1);
  Serial.print(", M2=");
  Serial.println(target_pos2);
  
  // Move to target position at 7.5 mm/s
  Serial.println("Starting priming movement at 7.5 mm/s...");
  if (!moveMotorsTo(target_pos1, target_pos2, 7.5)) {
    Serial.println("PRIMING FAILED!");
    return false;
  }
  
  arduino_pos1 = target_pos1;
  arduino_pos2 = target_pos2;
  
  motorsHomed = true;
  Serial.println("=== PRIMING SUCCESS ===");
  Serial.print("Final positions: M1=");
  Serial.print(arduino_pos1);
  Serial.print(", M2=");
  Serial.println(arduino_pos2);
  
  return true;
}

// ==================== STATE MACHINE FUNCTIONS ====================
bool executeLoad() {
  current_state = LOAD;
  
  if (!moveMotorsTo(LOAD_POSITION, LOAD_POSITION, MAX_SPEED_MMS)) {
    current_state = SAFE_MODE;
    return false;
  }
  
  return true;
}

bool executeSetup(float vol1, float vol2, float concentration) {
  current_state = SETUP;
  
  config.syringe_vol1 = vol1;
  config.syringe_vol2 = vol2;
  
  // Use concentration to set ratios, not volumes!
  // concentration is the percentage for syringe 1 (e.g., 70 means 70:30 ratio)
  config.ratio1 = concentration / 100.0;
  config.ratio2 = (100.0 - concentration) / 100.0;
  
  Serial.print("executeSetup: Concentration ");
  Serial.print(concentration);
  Serial.print("% -> Ratios: M1=");
  Serial.print(config.ratio1, 3);
  Serial.print(", M2=");
  Serial.println(config.ratio2, 3);
  
  config.remaining1 = config.syringe_vol1;
  config.remaining2 = config.syringe_vol2;
  config.dispensed1 = 0.0f;
  config.dispensed2 = 0.0f;
  
  return true;
}

bool executePrime() {
  current_state = PRIMED;
  
  config.prime_pos1 = arduino_pos1;
  config.prime_pos2 = arduino_pos2;
  
  return true;
}

// ==================== EXTRUSION VALIDATION ====================
ExtrusionValidation validateExtrusion(float total_volume_ml, float print_time_sec) {
  ExtrusionValidation result;
  result.is_valid = true;
  result.error_message = "";
  result.suggestion = "";
  
  // Boost system constants
  const float SLOW_SPEED_THRESHOLD = 1.0;  // mm/s - below this triggers boost
  const float BOOST_SPEED = 2.0;           // mm/s - speed during boost phase
  const float BOOST_DURATION = 1.0;        // seconds - duration of boost
  const float MIN_VIABLE_SPEED = 0.3;      // mm/s - below this motor stalls
  const float MIN_DISTANCE_FOR_BOOST = BOOST_SPEED * BOOST_DURATION * 1.1;  // 2.2mm with safety margin
  
  // Calculate volumes for each motor based on ratio
  float vol1_to_dispense = total_volume_ml * config.ratio1;
  float vol2_to_dispense = total_volume_ml * config.ratio2;
  
  // Calculate distances in mm
  float dist1_mm = vol1_to_dispense * MM_PER_ML;
  float dist2_mm = vol2_to_dispense * MM_PER_ML;
  
  // Calculate speeds without boost
  float speed1_mms = dist1_mm / print_time_sec;
  float speed2_mms = dist2_mm / print_time_sec;
  
  // Check each motor
  for (int motor = 1; motor <= 2; motor++) {
    float dist_mm = (motor == 1) ? dist1_mm : dist2_mm;
    float speed_mms = (motor == 1) ? speed1_mms : speed2_mms;
    float vol_ml = (motor == 1) ? vol1_to_dispense : vol2_to_dispense;
    float ratio = (motor == 1) ? config.ratio1 : config.ratio2;
    String motor_name = (motor == 1) ? "M1" : "M2";
    
    // Skip if motor doesn't need to move
    if (vol_ml < 0.01) continue;
    
    // Check if motor will need boost
    bool needs_boost = (speed_mms < SLOW_SPEED_THRESHOLD && print_time_sec > BOOST_DURATION);
    
    if (needs_boost) {
      // CASE 1: Needs boost but distance too short for boost phase
      if (dist_mm < MIN_DISTANCE_FOR_BOOST) {
        result.is_valid = false;
        result.error_message = motor_name + " boost failure";
        result.error_message += "\nDistance too short for boost";
        result.error_message += "\nNeeds: " + String(MIN_DISTANCE_FOR_BOOST, 1) + "mm";
        result.error_message += "\nHas: " + String(dist_mm, 1) + "mm";
        
        // Calculate suggestions
        float min_vol_needed = MIN_DISTANCE_FOR_BOOST / MM_PER_ML;
        float min_total_vol = min_vol_needed / ratio;
        
        // Time to avoid needing boost (speed >= 1.0 mm/s)
        float max_time_no_boost = dist_mm / SLOW_SPEED_THRESHOLD;
        
        result.suggestion = "Increase volume to >" + String(min_total_vol, 1) + "mL";
        result.suggestion += "\nOR decrease time to <" + String(max_time_no_boost, 0) + " sec";
        return result;
      }
      
      // CASE 2: Boost works, but phase 2 speed too slow
      float remaining_time = print_time_sec - BOOST_DURATION;
      float phase1_dist = BOOST_SPEED * BOOST_DURATION;
      float phase2_dist = dist_mm - phase1_dist;
      float phase2_speed = phase2_dist / remaining_time;
      
      if (phase2_speed < MIN_VIABLE_SPEED) {
        result.is_valid = false;
        result.error_message = motor_name + " phase 2 too slow";
        result.error_message += "\nAfter boost: " + String(phase2_speed, 2) + " mm/s";
        result.error_message += "\nMinimum: " + String(MIN_VIABLE_SPEED, 1) + " mm/s";
        
        // Calculate time needed for phase 2 to be viable
        float min_phase2_time = phase2_dist / MIN_VIABLE_SPEED;
        float max_total_time = BOOST_DURATION + min_phase2_time;
        
        // Calculate volume needed for phase 2 to be viable at current time
        float min_phase2_dist = MIN_VIABLE_SPEED * remaining_time;
        float min_total_dist = phase1_dist + min_phase2_dist;
        float min_vol_needed = min_total_dist / MM_PER_ML;
        float min_total_vol = min_vol_needed / ratio;
        
        result.suggestion = "Decrease time to <" + String(max_total_time, 0) + " sec";
        result.suggestion += "\nOR increase volume to >" + String(min_total_vol, 1) + "mL";
        return result;
      }
      
    } else {
      // CASE 3: Doesn't need boost, but speed too slow to move
      if (speed_mms < MIN_VIABLE_SPEED) {
        result.is_valid = false;
        result.error_message = motor_name + " speed too slow";
        result.error_message += "\nSpeed: " + String(speed_mms, 2) + " mm/s";
        result.error_message += "\nMinimum: " + String(MIN_VIABLE_SPEED, 1) + " mm/s";
        
        // Calculate max time for minimum viable speed
        float max_time = dist_mm / MIN_VIABLE_SPEED;
        
        result.suggestion = "Decrease time to <" + String(max_time, 0) + " sec";
        return result;
      }
    }
  }
  
  // All checks passed
  return result;
}

// Synchronized movement - BOTH finish at the same time
bool moveMotorsTimedSync(long target1, long target2, float speed1_mms, float speed2_mms, float duration_sec) {
  if (!syncPositionToTIC()) {
    current_state = SAFE_MODE;
    return false;
  }
  
  long start_pos1 = arduino_pos1;
  long start_pos2 = arduino_pos2;
  long dist1 = abs(target1 - start_pos1);
  long dist2 = abs(target2 - start_pos2);
  
  Serial.println("=== SYNCHRONIZED MOVEMENT ===");
  Serial.print("M1: ");
  Serial.print(start_pos1);
  Serial.print("→");
  Serial.print(target1);
  Serial.print(" (");
  Serial.print(dist1);
  Serial.print(" steps) @ ");
  Serial.print(speed1_mms, 3);
  Serial.println(" mm/s");
  Serial.print("M2: ");
  Serial.print(start_pos2);
  Serial.print("→");
  Serial.print(target2);
  Serial.print(" (");
  Serial.print(dist2);
  Serial.print(" steps) @ ");
  Serial.print(speed2_mms, 3);
  Serial.println(" mm/s");
  
  tic1.clearDriverError();
  tic2.clearDriverError();
  tic1.exitSafeStart();
  tic2.exitSafeStart();
  
  // Set speeds
  tic1.setMaxSpeed(stepsPerSecToTicUnits(mmsToStepsPerSec(speed1_mms)));
  tic2.setMaxSpeed(stepsPerSecToTicUnits(mmsToStepsPerSec(speed2_mms)));
  
  delay(50);
  
  // Start BOTH at the exact same time
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
    
    if (!m1_arrived && abs(pos1 - target1) <= 50) {
      m1_arrived = true;
      m1_finish_time = millis() - start_time;
      Serial.print("M1 arrived at ");
      Serial.print(m1_finish_time);
      Serial.println(" ms");
    }
    
    if (!m2_arrived && abs(pos2 - target2) <= 50) {
      m2_arrived = true;
      m2_finish_time = millis() - start_time;
      Serial.print("M2 arrived at ");
      Serial.print(m2_finish_time);
      Serial.println(" ms");
    }
    
    uint16_t err1 = tic1.getErrorStatus();
    uint16_t err2 = tic2.getErrorStatus();
    
    if (err1 != 0 || err2 != 0) {
      Serial.print("ERROR: M1=");
      Serial.print(err1);
      Serial.print(", M2=");
      Serial.println(err2);
      arduino_pos1 = pos1;
      arduino_pos2 = pos2;
      current_state = SAFE_MODE;
      return false;
    }
    
    delay(10);
  }
  
  unsigned long total_time = millis() - start_time;
  long time_diff = abs((long)m1_finish_time - (long)m2_finish_time);
  
  Serial.print("Both finished! Total time: ");
  Serial.print(total_time);
  Serial.print(" ms, Time difference: ");
  Serial.print(time_diff);
  Serial.println(" ms");
  
  arduino_pos1 = target1;
  arduino_pos2 = target2;
  
  return true;
}

bool executeExtrude(float total_volume_ml, float print_time_sec) {
  current_state = EXTRUDING;
  
  Serial.println("\n=== EXTRUSION START ===");
  Serial.print("Total volume requested: ");
  Serial.print(total_volume_ml, 2);
  Serial.print(" mL over ");
  Serial.print(print_time_sec, 1);
  Serial.println(" seconds");
  
  // Calculate volumes based on ratio
  // CRITICAL: Requested volume = TOTAL OUTPUT, distributed by ratio
  float vol1_to_dispense = total_volume_ml * config.ratio1;
  float vol2_to_dispense = total_volume_ml * config.ratio2;
  
  Serial.print("Ratio: M1=");
  Serial.print(config.ratio1 * 100, 0);
  Serial.print("% (");
  Serial.print(vol1_to_dispense, 3);
  Serial.print("mL), M2=");
  Serial.print(config.ratio2 * 100, 0);
  Serial.print("% (");
  Serial.print(vol2_to_dispense, 3);
  Serial.println("mL)");
  
  // SAFETY CHECK: Ensure sufficient volume in each syringe
  if (vol1_to_dispense > config.remaining1) {
    Serial.print("ERROR: M1 requires ");
    Serial.print(vol1_to_dispense, 2);
    Serial.print("mL but only ");
    Serial.print(config.remaining1, 2);
    Serial.println("mL remaining!");
    current_state = COMPLETE;
    return false;
  }
  
  if (vol2_to_dispense > config.remaining2) {
    Serial.print("ERROR: M2 requires ");
    Serial.print(vol2_to_dispense, 2);
    Serial.print("mL but only ");
    Serial.print(config.remaining2, 2);
    Serial.println("mL remaining!");
    current_state = COMPLETE;
    return false;
  }
  
  // Calculate speeds to finish at same time
  // speed = distance / time
  float dist1_mm = vol1_to_dispense * MM_PER_ML;
  float dist2_mm = vol2_to_dispense * MM_PER_ML;
  
  float speed1_mms = dist1_mm / print_time_sec;
  float speed2_mms = dist2_mm / print_time_sec;
  
  Serial.print("Distance: M1=");
  Serial.print(dist1_mm, 2);
  Serial.print("mm, M2=");
  Serial.print(dist2_mm, 2);
  Serial.println("mm");
  
  Serial.print("Initial calculated speeds: M1=");
  Serial.print(speed1_mms, 3);
  Serial.print(" mm/s, M2=");
  Serial.print(speed2_mms, 3);
  Serial.println(" mm/s");
  
  // ACCELERATION RAMP for slow motors (< 1.0 mm/s)
  // To overcome static friction, boost slow motors at startup
  const float SLOW_SPEED_THRESHOLD = 1.0;  // mm/s
  const float BOOST_SPEED = 2.0;           // mm/s for startup
  const float BOOST_DURATION = 1.0;        // seconds
  
  bool motor1_needs_boost = (speed1_mms < SLOW_SPEED_THRESHOLD && print_time_sec > BOOST_DURATION);
  bool motor2_needs_boost = (speed2_mms < SLOW_SPEED_THRESHOLD && print_time_sec > BOOST_DURATION);
  
  // If EITHER motor needs boost, BOTH motors do two-phase movement
  bool use_two_phase = (motor1_needs_boost || motor2_needs_boost) && (print_time_sec > BOOST_DURATION);
  
  float phase1_speed_m1, phase2_speed_m1;
  float phase1_speed_m2, phase2_speed_m2;
  long phase1_steps_m1, phase2_steps_m1;
  long phase1_steps_m2, phase2_steps_m2;
  
  if (use_two_phase) {
    Serial.println("=== TWO-PHASE MOVEMENT ===");
    
    // BOTH motors execute Phase 1 and Phase 2 together
    // Phase 1: BOOST_DURATION seconds
    // Phase 2: Remaining time
    
    float remaining_time = print_time_sec - BOOST_DURATION;
    
    // Motor 1 calculations
    if (motor1_needs_boost) {
      // M1 gets boost in Phase 1
      phase1_speed_m1 = BOOST_SPEED;
      float phase1_dist_m1 = BOOST_SPEED * BOOST_DURATION;
      float phase2_dist_m1 = dist1_mm - phase1_dist_m1;
      phase2_speed_m1 = phase2_dist_m1 / remaining_time;
      
      phase1_steps_m1 = (long)(phase1_dist_m1 / MM_PER_STEP);
      phase2_steps_m1 = (long)(phase2_dist_m1 / MM_PER_STEP);
      
      Serial.print("M1 BOOST: Phase1=");
      Serial.print(phase1_speed_m1, 2);
      Serial.print("mm/s (");
      Serial.print(phase1_dist_m1, 2);
      Serial.print("mm), Phase2=");
      Serial.print(phase2_speed_m1, 3);
      Serial.print("mm/s (");
      Serial.print(phase2_dist_m1, 2);
      Serial.println("mm)");
    } else {
      // M1 doesn't need boost - calculate speeds to match total distance/time
      // Distribute distance across two phases proportionally
      float phase1_dist_m1 = dist1_mm * (BOOST_DURATION / print_time_sec);
      float phase2_dist_m1 = dist1_mm - phase1_dist_m1;
      
      phase1_speed_m1 = phase1_dist_m1 / BOOST_DURATION;
      phase2_speed_m1 = phase2_dist_m1 / remaining_time;
      
      phase1_steps_m1 = (long)(phase1_dist_m1 / MM_PER_STEP);
      phase2_steps_m1 = (long)(phase2_dist_m1 / MM_PER_STEP);
      
      Serial.print("M1 NORMAL: Phase1=");
      Serial.print(phase1_speed_m1, 2);
      Serial.print("mm/s (");
      Serial.print(phase1_dist_m1, 2);
      Serial.print("mm), Phase2=");
      Serial.print(phase2_speed_m1, 3);
      Serial.print("mm/s (");
      Serial.print(phase2_dist_m1, 2);
      Serial.println("mm)");
    }
    
    // Motor 2 calculations
    if (motor2_needs_boost) {
      // M2 gets boost in Phase 1
      phase1_speed_m2 = BOOST_SPEED;
      float phase1_dist_m2 = BOOST_SPEED * BOOST_DURATION;
      float phase2_dist_m2 = dist2_mm - phase1_dist_m2;
      phase2_speed_m2 = phase2_dist_m2 / remaining_time;
      
      phase1_steps_m2 = (long)(phase1_dist_m2 / MM_PER_STEP);
      phase2_steps_m2 = (long)(phase2_dist_m2 / MM_PER_STEP);
      
      Serial.print("M2 BOOST: Phase1=");
      Serial.print(phase1_speed_m2, 2);
      Serial.print("mm/s (");
      Serial.print(phase1_dist_m2, 2);
      Serial.print("mm), Phase2=");
      Serial.print(phase2_speed_m2, 3);
      Serial.print("mm/s (");
      Serial.print(phase2_dist_m2, 2);
      Serial.println("mm)");
    } else {
      // M2 doesn't need boost - calculate speeds to match total distance/time
      float phase1_dist_m2 = dist2_mm * (BOOST_DURATION / print_time_sec);
      float phase2_dist_m2 = dist2_mm - phase1_dist_m2;
      
      phase1_speed_m2 = phase1_dist_m2 / BOOST_DURATION;
      phase2_speed_m2 = phase2_dist_m2 / remaining_time;
      
      phase1_steps_m2 = (long)(phase1_dist_m2 / MM_PER_STEP);
      phase2_steps_m2 = (long)(phase2_dist_m2 / MM_PER_STEP);
      
      Serial.print("M2 NORMAL: Phase1=");
      Serial.print(phase1_speed_m2, 2);
      Serial.print("mm/s (");
      Serial.print(phase1_dist_m2, 2);
      Serial.print("mm), Phase2=");
      Serial.print(phase2_speed_m2, 3);
      Serial.print("mm/s (");
      Serial.print(phase2_dist_m2, 2);
      Serial.println("mm)");
    }
  }
  
  // Calculate target positions (extrusion moves DOWN toward 1600)
  long steps1 = (long)(vol1_to_dispense * STEPS_PER_ML);
  long steps2 = (long)(vol2_to_dispense * STEPS_PER_ML);
  
  long target1 = arduino_pos1 - steps1;
  long target2 = arduino_pos2 - steps2;
  
  Serial.print("Current positions: M1=");
  Serial.print(arduino_pos1);
  Serial.print(", M2=");
  Serial.println(arduino_pos2);
  
  Serial.print("Steps to move: M1=");
  Serial.print(steps1);
  Serial.print(", M2=");
  Serial.println(steps2);
  
  Serial.print("Target positions: M1=");
  Serial.print(target1);
  Serial.print(", M2=");
  Serial.println(target2);
  
  // Safety check: don't go below 1600 (0mL position)
  if (target1 < 1600 || target2 < 1600) {
    Serial.println("ERROR: Would go below 0mL (position 1600)!");
    current_state = COMPLETE;
    return false;
  }
  
  // Execute movement with acceleration ramp if needed
  Serial.println("Starting synchronized movement...");
  
  if (use_two_phase) {
    // TWO-PHASE MOVEMENT - BOTH motors execute phases simultaneously
    Serial.println("=== EXECUTING SYNCHRONIZED TWO-PHASE MOVEMENT ===");
    
    // PHASE 1: Both motors move for exactly BOOST_DURATION
    long phase1_target_m1 = arduino_pos1 - phase1_steps_m1;
    long phase1_target_m2 = arduino_pos2 - phase1_steps_m2;
    
    Serial.print("Phase 1: ");
    Serial.print(BOOST_DURATION, 1);
    Serial.println(" seconds");
    Serial.print("  M1: ");
    Serial.print(arduino_pos1);
    Serial.print(" → ");
    Serial.print(phase1_target_m1);
    Serial.print(" @ ");
    Serial.print(phase1_speed_m1, 3);
    Serial.println(" mm/s");
    Serial.print("  M2: ");
    Serial.print(arduino_pos2);
    Serial.print(" → ");
    Serial.print(phase1_target_m2);
    Serial.print(" @ ");
    Serial.print(phase1_speed_m2, 3);
    Serial.println(" mm/s");
    
    // Execute Phase 1 with time sync
    if (!moveMotorsTimedSync(phase1_target_m1, phase1_target_m2, phase1_speed_m1, phase1_speed_m2, BOOST_DURATION)) {
      Serial.println("ERROR: Phase 1 failed!");
      current_state = SAFE_MODE;
      return false;
    }
    
    Serial.println("Phase 1 complete - both motors synchronized");
    
    // PHASE 2: Both motors move for exactly remaining time
    long phase2_target_m1 = target1;  // Final target
    long phase2_target_m2 = target2;  // Final target
    
    float remaining_time = print_time_sec - BOOST_DURATION;
    
    Serial.print("Phase 2: ");
    Serial.print(remaining_time, 1);
    Serial.println(" seconds");
    Serial.print("  M1: ");
    Serial.print(arduino_pos1);
    Serial.print(" → ");
    Serial.print(phase2_target_m1);
    Serial.print(" @ ");
    Serial.print(phase2_speed_m1, 3);
    Serial.println(" mm/s");
    Serial.print("  M2: ");
    Serial.print(arduino_pos2);
    Serial.print(" → ");
    Serial.print(phase2_target_m2);
    Serial.print(" @ ");
    Serial.print(phase2_speed_m2, 3);
    Serial.println(" mm/s");
    
    // Execute Phase 2 with time sync
    if (!moveMotorsTimedSync(phase2_target_m1, phase2_target_m2, phase2_speed_m1, phase2_speed_m2, remaining_time)) {
      Serial.println("ERROR: Phase 2 failed!");
      current_state = SAFE_MODE;
      return false;
    }
    
    Serial.println("Phase 2 complete - BOTH MOTORS FINISHED SIMULTANEOUSLY");
    
  } else {
    // SINGLE-PHASE MOVEMENT (no boost needed)
    Serial.println("=== EXECUTING SINGLE-PHASE MOVEMENT ===");
    
    Serial.print("Duration: ");
    Serial.print(print_time_sec, 1);
    Serial.println(" seconds");
    Serial.print("  M1 speed: ");
    Serial.print(speed1_mms, 3);
    Serial.println(" mm/s");
    Serial.print("  M2 speed: ");
    Serial.print(speed2_mms, 3);
    Serial.println(" mm/s");
    
    // Execute with time sync
    if (!moveMotorsTimedSync(target1, target2, speed1_mms, speed2_mms, print_time_sec)) {
      Serial.println("ERROR: Movement failed!");
      current_state = SAFE_MODE;
      return false;
    }
    
    Serial.println("Movement complete - both motors synchronized");
  }
  
  // Update tracking
  config.dispensed1 += vol1_to_dispense;
  config.dispensed2 += vol2_to_dispense;
  config.remaining1 -= vol1_to_dispense;
  config.remaining2 -= vol2_to_dispense;
  
  Serial.println("=== EXTRUSION COMPLETE ===");
  Serial.print("Total dispensed: M1=");
  Serial.print(config.dispensed1, 2);
  Serial.print("mL, M2=");
  Serial.print(config.dispensed2, 2);
  Serial.println("mL");
  
  Serial.print("Remaining: M1=");
  Serial.print(config.remaining1, 2);
  Serial.print("mL, M2=");
  Serial.print(config.remaining2, 2);
  Serial.println("mL\n");
  
  return true;
}

// ==================== UI DRAWING FUNCTIONS ====================
void drawMotorZeroCheckPage() {
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  
  display.setCursor(40, 150);
  display.print("MOTOR SETUP");
  
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(60, 250);
  display.print("Are the motors at");
  display.setCursor(80, 285);
  display.print("zero position?");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 350);
  display.print("Zero = fully retracted");
  display.setCursor(50, 380);
  display.print("(no load on syringes)");
  
  // YES button - large green
  display.fillRoundRect(50, 480, 180, 120, 10, CONFIRM_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(110, 555);
  display.print("YES");
  
  // NO button - large red
  display.fillRoundRect(250, 480, 180, 120, 10, CANCEL_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(320, 555);
  display.print("NO");
  
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(50, 650);
  display.print("If NO: Motors will retract");
  display.setCursor(50, 680);
  display.print("slowly for 15 seconds");
}

void drawRetractingToZeroPage() {
  unsigned long elapsed = millis() - retractionStartTime;
  unsigned long remaining = RETRACTION_DURATION - elapsed;
  float progress = (float)elapsed / (float)RETRACTION_DURATION * 100.0;
  
  if (progress > 100.0) progress = 100.0;
  
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  
  display.setCursor(40, 150);
  display.print("RETRACTING");
  display.setCursor(90, 190);
  display.print("TO ZERO");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(80, 280);
  display.print("Moving motors slowly...");
  display.setCursor(100, 310);
  display.print("Please wait");
  
  // Progress bar
  display.drawRect(50, 380, 380, 40, TEXT_COLOR);
  int fillWidth = (int)(370.0 * progress / 100.0);
  display.fillRect(55, 385, fillWidth, 30, CONFIRM_COLOR);
  
  // Time remaining
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(140, 470);
  display.print(remaining / 1000);
  display.print(" sec");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(120, 530);
  display.print("Progress: ");
  display.print((int)progress);
  display.print("%");
  
  display.setCursor(50, 630);
  display.print("Slow speed: 0.5 mm/s");
  display.setCursor(50, 660);
  display.print("Safe retraction in progress");
}

void drawCalibrationInProgressPage() {
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  
  display.setCursor(60, 200);
  display.print("CALIBRATING");
  display.setCursor(130, 240);
  display.print("RANGE");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(80, 320);
  display.print("Establishing 0-15000");
  display.setCursor(100, 350);
  display.print("reference range");
  
  display.setCursor(50, 430);
  display.print("Step 1: Moving to MIN (0)");
  display.setCursor(50, 470);
  display.print("Step 2: Moving to MAX (15000)");
  
  display.setCursor(100, 560);
  display.print("Please wait...");
  
  display.setCursor(80, 650);
  display.print("Current positions:");
  display.setCursor(80, 680);
  display.print("M1: ");
  display.print(arduino_pos1);
  display.print("  M2: ");
  display.print(arduino_pos2);
}

void drawShutdownConfirmPage() {
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(CANCEL_COLOR);
  
  display.setCursor(80, 150);
  display.print("SHUTDOWN?");
  
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(60, 250);
  display.print("This will:");
  display.setCursor(60, 285);
  display.print("1. Turn off heaters");
  display.setCursor(60, 320);
  display.print("2. Retract motors to zero");
  display.setCursor(60, 355);
  display.print("3. Prepare for power off");
  
  display.setCursor(60, 420);
  display.print("Continue with shutdown?");
  
  // YES button (red, left)
  display.fillRoundRect(50, 520, 160, 100, 10, CANCEL_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(100, 585);
  display.print("YES");
  
  // NO button (green, right)
  display.fillRoundRect(270, 520, 160, 100, 10, CONFIRM_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(330, 585);
  display.print("NO");
}

void drawShuttingDownPage() {
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  
  display.setCursor(60, 150);
  display.print("SHUTTING");
  display.setCursor(120, 200);
  display.print("DOWN");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(80, 300);
  display.print("Heaters: OFF");
  
  display.setCursor(80, 350);
  display.print("Retracting motors...");
  
  display.setCursor(80, 430);
  display.print("Current positions:");
  display.setCursor(80, 465);
  display.print("M1: ");
  display.print(arduino_pos1);
  display.print(" steps");
  display.setCursor(80, 500);
  display.print("M2: ");
  display.print(arduino_pos2);
  display.print(" steps");
  
  display.setCursor(80, 570);
  display.print("Target: 0 steps (zero)");
  
  display.setCursor(60, 650);
  display.print("Please wait - do not power off");
}

void drawShutdownCompletePage() {
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(CONFIRM_COLOR);
  
  display.setCursor(70, 200);
  display.print("SHUTDOWN");
  display.setCursor(80, 250);
  display.print("COMPLETE");
  
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(TEXT_COLOR);
  
  display.setCursor(80, 350);
  display.print("Motors: Retracted to zero");
  display.setCursor(80, 385);
  display.print("Heaters: OFF");
  display.setCursor(80, 420);
  display.print("System: Safe");
  
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(60, 520);
  display.print("Safe to power off Arduino");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(80, 600);
  display.print("You may now disconnect power");
}

void drawWelcomePage() {
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  
  display.setCursor(80, 200);
  display.print("BioPrint AM");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(120, 300);
  display.print("v3.1 Integrated");
  
  display.setCursor(70, 400);
  display.print("Temperature Control");
  
  display.setCursor(50, 450);
  display.print("Syringe Range:");
  display.setCursor(50, 475);
  display.print("0mL=1600  10mL=11600");
  
  display.setCursor(80, 520);
  display.print("Heat Mats:");
  display.setCursor(80, 545);
  display.print("A0: ");
  display.print(currentTemperatures[0], 1);
  display.print(" C  A1: ");
  display.print(currentTemperatures[1], 1);
  display.print(" C");
  
  display.setCursor(80, 560);
  if (motorsHomed) {
    display.setTextColor(CONFIRM_COLOR);
    display.print("Motors: HOMED");
  } else {
    display.setTextColor(CANCEL_COLOR);
    display.print("Motors: NOT HOMED!");
  }
  
  display.setTextColor(TEXT_COLOR);
  display.fillRoundRect(140, 630, 200, 100, 10, BUTTON_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(185, 690);
  display.print("START");
}

void drawHomingPage() {
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(100, 200);
  display.print("HOMING...");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 280);
  display.print("1. Moving to load position");
  display.setCursor(65, 310);
  display.print("(15000 steps)");
  
  display.setCursor(50, 360);
  display.print("2. Moving to volume position");
  if (selectedVol1 > 0 && selectedVol2 > 0) {
    long target1 = 1600 + mlToSteps(selectedVol1);
    long target2 = 1600 + mlToSteps(selectedVol2);
    display.setCursor(65, 390);
    display.print("M1: ");
    display.print(selectedVol1, 1);
    display.print("mL -> pos ");
    display.print(target1);
    display.setCursor(65, 420);
    display.print("M2: ");
    display.print(selectedVol2, 1);
    display.print("mL -> pos ");
    display.print(target2);
  } else {
    display.setCursor(65, 390);
    display.print("(10mL = position 11600)");
  }
  
  display.setCursor(80, 480);
  display.print("Please wait...");
  
  display.setCursor(80, 540);
  display.print("M1: ");
  display.print(arduino_pos1);
  display.print(" steps");
  display.setCursor(80, 570);
  display.print("M2: ");
  display.print(arduino_pos2);
  display.print(" steps");
}

void drawLoadingSyringesPage() {
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(40, 150);
  display.print("LOAD SYRINGES");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 230);
  display.print("Instructions:");
  
  display.setCursor(50, 270);
  display.print("1. Press BEGIN HOMING");
  display.setCursor(50, 300);
  display.print("2. Wait for motors (15000)");
  display.setCursor(50, 330);
  display.print("3. Insert & fill syringes");
  display.setCursor(50, 360);
  display.print("4. Secure in holders");
  display.setCursor(50, 390);
  display.print("5. Motors move to volume");
  
  display.setCursor(50, 450);
  display.print("Target volumes:");
  if (selectedVol1 > 0 && selectedVol2 > 0) {
    display.setCursor(50, 480);
    display.print("M1: ");
    display.print(selectedVol1, 1);
    display.print("mL");
    display.setCursor(50, 510);
    display.print("M2: ");
    display.print(selectedVol2, 1);
    display.print("mL");
  } else {
    display.setCursor(50, 480);
    display.print("M1: 10.0 mL (max)");
    display.setCursor(50, 510);
    display.print("M2: 10.0 mL (max)");
  }
  
  display.fillRoundRect(90, 580, 300, 80, 10, CONFIRM_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(130, 630);
  display.print("BEGIN");
  display.setCursor(125, 660);
  display.print("HOMING");
  
  display.setFont(&FreeSans9pt7b);
  display.fillRoundRect(40, 700, 120, 70, 10, CANCEL_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(70, 745);
  display.print("BACK");
  
  display.fillRoundRect(320, 700, 120, 70, 10, CLEAR_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(345, 745);
  display.print("HOME");
}

void drawWaitingForSyringesPage() {
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(40, 150);
  display.print("LOAD SYRINGES");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 230);
  display.setTextColor(CONFIRM_COLOR);
  display.print("Motors at loading position!");
  display.setTextColor(TEXT_COLOR);
  display.setCursor(50, 260);
  display.print("(15000 steps)");
  
  display.setCursor(50, 330);
  display.print("Now:");
  display.setCursor(50, 365);
  display.print("1. Insert syringes");
  display.setCursor(50, 395);
  display.print("2. Fill with material");
  display.setCursor(50, 425);
  display.print("3. Secure in holders");
  
  display.setCursor(50, 490);
  display.print("Target volumes:");
  if (selectedVol1 > 0 && selectedVol2 > 0) {
    long target1 = 1600 + mlToSteps(selectedVol1);
    long target2 = 1600 + mlToSteps(selectedVol2);
    display.setCursor(50, 520);
    display.print("M1: ");
    display.print(selectedVol1, 1);
    display.print("mL (pos ");
    display.print(target1);
    display.print(")");
    display.setCursor(50, 550);
    display.print("M2: ");
    display.print(selectedVol2, 1);
    display.print("mL (pos ");
    display.print(target2);
    display.print(")");
  } else {
    display.setCursor(50, 520);
    display.print("M1: 10.0 mL (pos 11600)");
    display.setCursor(50, 550);
    display.print("M2: 10.0 mL (pos 11600)");
  }
  
  display.fillRoundRect(90, 600, 300, 80, 10, CONFIRM_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(110, 650);
  display.print("SYRINGES");
  display.setCursor(145, 680);
  display.print("LOADED");
  
  display.setFont(&FreeSans9pt7b);
  display.fillRoundRect(170, 710, 140, 60, 10, CLEAR_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(205, 748);
  display.print("HOME");
}

void drawHomePage() {
  display.fillScreen(BG_COLOR);
  display.setTextColor(TEXT_COLOR);
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(35, 50);
  display.print("PARAMETER CONTROL");
  
  drawHomeButton(40, 120, "Temperature", selectedTemp, "C");
  drawHomeButton(40, 230, "Volume 1", selectedVol1, "mL");
  drawHomeButton(40, 340, "Volume 2", selectedVol2, "mL");
  drawHomeButton(40, 450, "Concentration", selectedConc, "%");
  
  // CONTINUE button (left)
  display.fillRoundRect(40, 600, 180, 80, 8, CONFIRM_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(60, 650);
  display.print("CONTINUE");
  
  // SHUTDOWN button (right, red)
  display.fillRoundRect(260, 600, 180, 80, 8, CANCEL_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(270, 650);
  display.print("SHUTDOWN");
}

void drawHomeButton(int x, int y, const char* label, float value, const char* unit) {
  display.fillRoundRect(x, y, 400, 90, 8, BUTTON_COLOR);
  
  display.setFont(&FreeSansBold12pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(x + 15, y + 30);
  display.print(label);
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(x + 15, y + 65);
  if (value < 0) {
    display.print("Not Selected");
  } else {
    display.print("Selected: ");
    
    if (unit[0] == '%') {
      float inverse = 100 - value;
      display.print(value, 1);
      display.print(":");
      display.print(inverse, 1);
      display.print("%");
    } else {
      display.print(value, 1);
      display.print(" ");
      if (unit[0] == 'C') {
        display.write(248);
      }
      display.print(unit);
    }
  }
}

void drawParameterPage(int* options, int numOptions, float currentValue, const char* title, const char* unit) {
  display.fillScreen(BG_COLOR);
  display.setTextColor(TEXT_COLOR);
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(50, 40);
  display.print("Select ");
  display.print(title);
  display.print(":");
  
  int boxX = 40;
  int boxY = 75;
  int boxWidth = 260;
  int boxHeight = 100;
  int buttonSize = 70;
  int buttonSpacing = 10;
  
  display.fillRoundRect(boxX, boxY, buttonSize, boxHeight, 8, CONFIRM_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(boxX + 28, boxY + 62);
  display.print("-");
  
  display.fillRoundRect(boxX + buttonSize + buttonSpacing, boxY, boxWidth, boxHeight, 8, BUTTON_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  
  if (tempSelection < 0) {
    display.setCursor(boxX + buttonSize + buttonSpacing + 45, boxY + 62);
    display.print("Not Set");
  } else {
    String valueStr = "";
    
    if (unit[0] == '%') {
      float inverse = 100 - tempSelection;
      valueStr = String(tempSelection, 0) + ":" + String(inverse, 0) + "%";
      int charCount = valueStr.length();
      int textWidth = charCount * 18;
      int textX = boxX + buttonSize + buttonSpacing + (boxWidth / 2) - (textWidth / 2);
      display.setCursor(textX, boxY + 62);
      display.print(valueStr);
    } else if (unit[0] == 'C') {
      int charCount = String(tempSelection, 1).length() + 2;
      int textWidth = charCount * 18;
      int textX = boxX + buttonSize + buttonSpacing + (boxWidth / 2) - (textWidth / 2);
      display.setCursor(textX, boxY + 62);
      display.print(tempSelection, 1);
      display.print(" ");
      display.write(248);
      display.print(unit);
    } else {
      valueStr = String(tempSelection, 1) + " " + unit;
      int charCount = valueStr.length();
      int textWidth = charCount * 18;
      int textX = boxX + buttonSize + buttonSpacing + (boxWidth / 2) - (textWidth / 2);
      display.setCursor(textX, boxY + 62);
      display.print(valueStr);
    }
  }
  
  display.fillRoundRect(boxX + buttonSize + buttonSpacing + boxWidth + buttonSpacing, boxY, buttonSize, boxHeight, 8, CONFIRM_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(boxX + buttonSize + buttonSpacing + boxWidth + buttonSpacing + 24, boxY + 62);
  display.print("+");
  
  int cols = 3;
  int buttonWidth = 120;
  int buttonHeight = 80;
  int spacingX = 20;
  int spacingY = 20;
  int startX = 40;
  int startY = 200;
  
  for (int i = 0; i < numOptions; i++) {
    int row = i / cols;
    int col = i % cols;
    int bx = startX + col * (buttonWidth + spacingX);
    int by = startY + row * (buttonHeight + spacingY);
    
    bool isSelected = (abs(options[i] - tempSelection) < 0.01);
    uint16_t bgColor = isSelected ? BUTTON_SELECTED_COLOR : BUTTON_COLOR;
    
    display.fillRoundRect(bx, by, buttonWidth, buttonHeight, 8, bgColor);
    display.setFont(&FreeSansBold18pt7b);
    display.setTextColor(BUTTON_TEXT_COLOR);
    
    String valueStr = String(options[i]);
    int charCount = valueStr.length();
    int textWidth = charCount * 20;
    int textX = bx + (buttonWidth - textWidth) / 2;
    int textY = by + 52;
    
    display.setCursor(textX, textY);
    display.print(options[i]);
  }
  
  display.fillRoundRect(40, 700, 120, 70, 8, CANCEL_COLOR);
  display.setFont(&FreeSansBold12pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(65, 745);
  display.print("BACK");
  
  display.fillRoundRect(180, 700, 120, 70, 8, CLEAR_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(205, 745);
  display.print("HOME");
  
  display.fillRoundRect(320, 700, 120, 70, 8, CONFIRM_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(330, 745);
  display.print("CONFIRM");
}

void drawPrintConfirmPage() {
  display.fillScreen(BG_COLOR);
  display.setTextColor(TEXT_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setCursor(80, 50);
  display.print("Begin Print?");
  
  int yPos = 130;
  int lineHeight = 90;
  
  drawParameterSummary(40, yPos, "Temperature:", selectedTemp, "C");
  yPos += lineHeight;
  
  drawParameterSummary(40, yPos, "Volume 1:", selectedVol1, "mL");
  yPos += lineHeight;
  
  drawParameterSummary(40, yPos, "Volume 2:", selectedVol2, "mL");
  yPos += lineHeight;
  
  drawParameterSummary(40, yPos, "Concentration:", selectedConc, "%");
  yPos += lineHeight;
  
  drawParameterSummary(40, yPos, "Speed:", selectedSpeed, "mm/s");
  
  display.fillRoundRect(140, 630, 200, 70, 8, CONFIRM_COLOR);
  display.setFont(&FreeSansBold12pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(170, 675);
  display.print("CONFIRM");
  
  display.fillRoundRect(40, 720, 120, 50, 8, CANCEL_COLOR);
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(70, 753);
  display.print("BACK");
  
  display.fillRoundRect(320, 720, 120, 50, 8, CLEAR_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(345, 753);
  display.print("HOME");
}

void drawParameterSummary(int x, int y, const char* label, float value, const char* unit) {
  display.setFont(&FreeSansBold12pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(x, y);
  display.print(label);
  
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(x, y + 40);
  if (value < 0) {
    display.print("Not Selected");
  } else {
    if (unit[0] == '%') {
      float inverse = 100 - value;
      display.print(value, 1);
      display.print(":");
      display.print(inverse, 1);
      display.print("%");
    } else {
      display.print(value, 1);
      display.print(" ");
      if (unit[0] == 'C') {
        display.write(248);
      }
      display.print(unit);
    }
  }
}

void drawLoadingPage() {
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(80, 150);
  display.print("LOADING...");
  
  display.setFont(&FreeSans9pt7b);
  
  display.setCursor(50, 200);
  display.print("Syringe Temperature:");
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(50, 235);
  if (currentDisplayTemp > -999.0) {
    display.print(currentDisplayTemp, 1);
    display.print(" C / ");
    display.print(Setpoint_Syringe, 0);
    display.print(" C");
  } else {
    display.print("--- C");
  }
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 280);
  display.print("Heat Mat Temperature:");
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(50, 315);
  if (Input_HeatMat > -999.0) {
    display.print(Input_HeatMat, 1);
    display.print(" C / 80 C");
  } else {
    display.print("--- C");
  }
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 370);
  display.print("PWM Values:");
  display.setCursor(50, 400);
  display.print("Heat Mat PID: ");
  display.print((Output_HeatMat / 255.0) * 100.0, 1);
  display.print("%");
  display.setCursor(50, 430);
  display.print("Syringe PID: ");
  display.print((Output_Syringe / 255.0) * 100.0, 1);
  display.print("%");
  
  display.setCursor(50, 470);
  display.setTextColor(CONFIRM_COLOR);
  if (!syringesTempReached) {
    display.print("Phase 1: Heating to 80C");
  } else {
    display.print("Phase 2: Modulating");
  }
  display.setTextColor(TEXT_COLOR);
  
  display.setCursor(50, 520);
  display.print("A0:");
  display.print(currentTemperatures[0], 1);
  display.print(" A1:");
  display.print(currentTemperatures[1], 1);
  
  display.setCursor(50, 550);
  display.print("A2:");
  display.print(currentTemperatures[2], 1);
  display.print(" A3:");
  display.print(currentTemperatures[3], 1);
  
  display.fillRoundRect(140, 680, 200, 80, 10, CANCEL_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(185, 730);
  display.print("HOME");
}

void drawTempReadyPage() {
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(80, 120);
  display.print("READY?");
  
  display.setFont(&FreeSans9pt7b);
  
  display.setCursor(50, 200);
  display.print("Syringe Temperature:");
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(50, 240);
  if (currentDisplayTemp > -999.0) {
    display.print(currentDisplayTemp, 1);
    display.print(" C / ");
    display.print(Setpoint_Syringe, 0);
    display.print(" C");
  } else {
    display.print("--- C");
  }
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 290);
  display.print("Heat Mat: ");
  if (Input_HeatMat > -999.0) {
    display.print(Input_HeatMat, 1);
    display.print(" C");
  } else {
    display.print("--- C");
  }
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 350);
  
  if (systemReady) {
    display.setTextColor(CONFIRM_COLOR);
    display.print("Temperature reached!");
    display.setTextColor(TEXT_COLOR);
    display.setCursor(50, 380);
    display.print("Actuators initialized");
    display.setCursor(50, 410);
    display.print("Motors homed");
  } else {
    display.print("Waiting for:");
    int yPos = 380;
    if (abs(currentDisplayTemp - Setpoint_Syringe) > TEMP_TOLERANCE) {
      display.setCursor(50, yPos);
      display.print("- Syringe temperature");
      yPos += 30;
    }
    if (current_state != PRIMED) {
      display.setCursor(50, yPos);
      display.print("- Actuator initialization");
      yPos += 30;
    }
    if (!motorsHomed) {
      display.setCursor(50, yPos);
      display.setTextColor(CANCEL_COLOR);
      display.print("- MOTOR HOMING REQUIRED!");
      display.setTextColor(TEXT_COLOR);
      display.setCursor(50, yPos + 25);
      display.print("  (Press HOME on welcome page)");
    }
  }
  
  display.setCursor(50, 510);
  display.print("A0=");
  display.print(currentTemperatures[0], 1);
  display.print(" A1=");
  display.print(currentTemperatures[1], 1);
  
  display.setCursor(50, 540);
  display.print("A2=");
  display.print(currentTemperatures[2], 1);
  display.print(" A3=");
  display.print(currentTemperatures[3], 1);
  
  if (systemReady) {
    display.fillRoundRect(90, 600, 300, 80, 10, CONFIRM_COLOR);
    display.setFont(&FreeSansBold18pt7b);
    display.setTextColor(BUTTON_TEXT_COLOR);
    display.setCursor(180, 655);
    display.print("YES");
  }
  
  display.setFont(&FreeSans9pt7b);
  display.fillRoundRect(140, 700, 200, 60, 10, CANCEL_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(190, 740);
  display.print("HOME");
}

void drawExtrusionSetupPage() {
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(40, 80);
  display.print("EXTRUSION");
  display.setCursor(100, 120);
  display.print("SETUP");
  
  display.setFont(&FreeSans9pt7b);
  
  // Volume to extrude section
  display.setCursor(50, 180);
  display.print("Volume to Extrude:");
  
  display.fillRoundRect(50, 200, 80, 70, 8, CONFIRM_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(80, 245);
  display.print("-");
  
  display.fillRoundRect(150, 200, 180, 70, 8, BUTTON_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(185, 245);
  display.print(extrusionVolume, 1);
  display.setFont(&FreeSans9pt7b);
  display.print(" mL");
  
  display.fillRoundRect(350, 200, 80, 70, 8, CONFIRM_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(378, 245);
  display.print("+");
  
  // Print Time section
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(50, 310);
  display.print("Print Time:");
  
  display.fillRoundRect(50, 330, 80, 70, 8, CONFIRM_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(80, 375);
  display.print("-");
  
  display.fillRoundRect(150, 330, 180, 70, 8, BUTTON_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(185, 375);
  display.print((int)printTime);
  display.setFont(&FreeSans9pt7b);
  display.print(" sec");
  
  display.fillRoundRect(350, 330, 80, 70, 8, CONFIRM_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(378, 375);
  display.print("+");
  
  // Info section
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(50, 440);
  display.print("Remaining:");
  display.setCursor(50, 470);
  display.print("M1: ");
  display.print(config.remaining1, 2);
  display.print(" mL");
  display.setCursor(50, 500);
  display.print("M2: ");
  display.print(config.remaining2, 2);
  display.print(" mL");
  
  // Calculate required volumes
  float reqVol1 = extrusionVolume * config.ratio1;
  float reqVol2 = extrusionVolume * config.ratio2;
  
  display.setCursor(50, 540);
  display.print("Required: M1=");
  display.print(reqVol1, 2);
  display.print(" M2=");
  display.print(reqVol2, 2);
  
  // Buttons
  display.fillRoundRect(90, 600, 300, 80, 10, CONFIRM_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(130, 655);
  display.print("START");
  
  display.setFont(&FreeSans9pt7b);
  display.fillRoundRect(140, 700, 200, 60, 10, CANCEL_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(190, 740);
  display.print("HOME");
}

void drawPostExtrusionOptionsPage() {
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(40, 100);
  display.print("EXTRUSION");
  display.setCursor(80, 140);
  display.print("COMPLETE!");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 220);
  display.print("Dispensed this cycle:");
  display.setCursor(50, 250);
  display.print("M1: ");
  float cycle_disp1 = config.dispensed1 - cycleStartDispensed1;
  display.print(cycle_disp1, 2);
  display.print(" mL");
  display.setCursor(50, 280);
  display.print("M2: ");
  float cycle_disp2 = config.dispensed2 - cycleStartDispensed2;
  display.print(cycle_disp2, 2);
  display.print(" mL");
  
  display.setCursor(50, 330);
  display.print("Remaining:");
  display.setCursor(50, 360);
  display.print("M1: ");
  display.print(config.remaining1, 2);
  display.print(" mL");
  display.setCursor(50, 390);
  display.print("M2: ");
  display.print(config.remaining2, 2);
  display.print(" mL");
  
  // Three button options
  display.fillRoundRect(40, 450, 120, 70, 10, CONFIRM_COLOR);
  display.setFont(&FreeSansBold12pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(55, 495);
  display.print("SAME");
  
  display.fillRoundRect(180, 450, 120, 70, 10, BUTTON_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(190, 495);
  display.print("ADJUST");
  
  display.fillRoundRect(320, 450, 120, 70, 10, CLEAR_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(335, 495);
  display.print("FINISH");
  
  display.setFont(&FreeSans9pt7b);
  display.fillRoundRect(140, 600, 200, 70, 10, CANCEL_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(190, 645);
  display.print("HOME");
}

void drawReadyToPrintPage() {
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(60, 100);
  display.print("READY!");
  
  display.setFont(&FreeSans9pt7b);
  
  display.setCursor(50, 180);
  display.print("Temperature: ");
  display.print(Input_Syringe, 1);
  display.print(" / ");
  display.print(Setpoint_Syringe, 0);
  display.print(" C");
  
  display.setCursor(50, 230);
  display.print("Remaining:");
  display.setCursor(50, 260);
  display.print("M1: ");
  display.print(config.remaining1, 2);
  display.print(" mL");
  display.setCursor(50, 290);
  display.print("M2: ");
  display.print(config.remaining2, 2);
  display.print(" mL");
  
  display.setCursor(50, 340);
  display.print("Dispensed:");
  display.setCursor(50, 370);
  display.print("M1: ");
  display.print(config.dispensed1, 2);
  display.print(" mL");
  display.setCursor(50, 400);
  display.print("M2: ");
  display.print(config.dispensed2, 2);
  display.print(" mL");
  
  display.fillRoundRect(90, 500, 300, 120, 10, CONFIRM_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(155, 575);
  display.print("PRINT");
  
  display.setFont(&FreeSans9pt7b);
  display.fillRoundRect(140, 670, 200, 70, 10, CANCEL_COLOR);
  display.setCursor(190, 715);
  display.print("HOME");
}

void drawPrintingPage() {
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(80, 100);
  display.print("PRINTING");
  
  display.setFont(&FreeSans9pt7b);
  
  display.setCursor(50, 160);
  display.print("Current Temperature:");
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(50, 195);
  if (currentDisplayTemp > -999.0) {
    display.print(currentDisplayTemp, 1);
    display.print(" C");
  } else {
    display.print("--- C");
  }
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 235);
  display.print("Selected Temperature:");
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(50, 270);
  display.print(Setpoint_Syringe, 1);
  display.print(" C");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 320);
  display.print("Remaining:");
  display.setCursor(50, 350);
  display.print("M1: ");
  display.print(config.remaining1, 2);
  display.print(" mL");
  display.setCursor(50, 380);
  display.print("M2: ");
  display.print(config.remaining2, 2);
  display.print(" mL");
  
  display.setCursor(50, 430);
  display.print("This Cycle:");
  display.setCursor(50, 460);
  display.print("M1: ");
  float cycle_disp1 = config.dispensed1 - cycleStartDispensed1;
  display.print(cycle_disp1, 2);
  display.print(" / ");
  display.print(cycleTargetVol1, 2);
  display.print(" mL");
  display.setCursor(50, 490);
  display.print("M2: ");
  float cycle_disp2 = config.dispensed2 - cycleStartDispensed2;
  display.print(cycle_disp2, 2);
  display.print(" / ");
  display.print(cycleTargetVol2, 2);
  display.print(" mL");
  
  // Calculate progress based on PRIMARY syringe (whichever has larger target)
  float progress = 0.0;
  if (cycleTargetVol1 >= cycleTargetVol2 && cycleTargetVol1 > 0) {
    progress = (cycle_disp1 / cycleTargetVol1) * 100.0;
  } else if (cycleTargetVol2 > 0) {
    progress = (cycle_disp2 / cycleTargetVol2) * 100.0;
  }
  progress = constrain(progress, 0.0, 100.0);
  
  display.setCursor(50, 530);
  display.print("Cycle Progress: ");
  display.print(progress, 0);
  display.print(" %");
  
  display.fillRoundRect(140, 600, 200, 80, 10, CANCEL_COLOR);
  display.setFont(&FreeSansBold12pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(165, 650);
  display.print("STOP");
}

void drawPrintDonePage() {
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(CONFIRM_COLOR);
  display.setCursor(60, 200);
  display.print("PRINT DONE!");
  
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(TEXT_COLOR);
  
  display.setCursor(80, 300);
  display.print("Total Dispensed:");
  display.setCursor(80, 340);
  display.print("M1: ");
  display.print(config.dispensed1, 2);
  display.print(" mL");
  display.setCursor(80, 370);
  display.print("M2: ");
  display.print(config.dispensed2, 2);
  display.print(" mL");
  
  display.setCursor(80, 430);
  display.print("Final Temperature:");
  display.setCursor(80, 460);
  display.print(currentDisplayTemp, 1);
  display.print(" C");
  
  // FINISH button (return to start)
  display.fillRoundRect(40, 550, 180, 80, 10, CLEAR_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(85, 605);
  display.print("FINISH");
  
  // HOME button
  display.fillRoundRect(260, 550, 180, 80, 10, CONFIRM_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(310, 605);
  display.print("HOME");
}

void drawErrorPage(const char* message) {
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(CANCEL_COLOR);
  display.setCursor(100, 200);
  display.print("ERROR");
  
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(80, 300);
  display.print(message);
  
  display.fillRoundRect(165, 450, 150, 70, 10, BUTTON_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(205, 495);
  display.print("HOME");
}

void drawValidationErrorPage(String error_msg, String suggestions) {
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(CANCEL_COLOR);
  display.setCursor(50, 100);
  display.print("CANNOT");
  display.setCursor(80, 140);
  display.print("EXTRUDE");
  
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(TEXT_COLOR);
  
  // Parse and display error message (split by newlines)
  int yPos = 220;
  int lineStart = 0;
  for (int i = 0; i <= error_msg.length(); i++) {
    if (i == error_msg.length() || error_msg.charAt(i) == '\n') {
      String line = error_msg.substring(lineStart, i);
      display.setCursor(50, yPos);
      display.print(line);
      yPos += 30;
      lineStart = i + 1;
    }
  }
  
  // Suggestions section
  yPos += 20;
  display.setTextColor(CONFIRM_COLOR);
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(50, yPos);
  display.print("Suggestions:");
  
  yPos += 40;
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(TEXT_COLOR);
  
  // Parse suggestions
  lineStart = 0;
  for (int i = 0; i <= suggestions.length(); i++) {
    if (i == suggestions.length() || suggestions.charAt(i) == '\n') {
      String line = suggestions.substring(lineStart, i);
      display.setCursor(50, yPos);
      display.print(line);
      yPos += 30;
      lineStart = i + 1;
    }
  }
  
  // BACK button
  display.fillRoundRect(140, 680, 200, 70, 10, BUTTON_COLOR);
  display.setFont(&FreeSansBold12pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(180, 725);
  display.print("BACK");
}

// ==================== TOUCH HANDLERS ====================
void handleMotorZeroCheckTouch(int x, int y) {
  // YES button
  if (x >= 50 && x <= 230 && y >= 480 && y <= 600) {
    Serial.println("User confirmed motors at zero");
    currentPage = CALIBRATION_IN_PROGRESS;
    drawCalibrationInProgressPage();
    delay(500);  // Show calibration page briefly
    
    // Run calibration
    calibrateMotorsOnStartup();
    calibrationComplete = true;
    
    // Go to welcome page
    currentPage = WELCOME;
    drawWelcomePage();
    return;
  }
  
  // NO button
  if (x >= 250 && x <= 430 && y >= 480 && y <= 600) {
    Serial.println("User indicated motors NOT at zero - starting retraction");
    currentPage = RETRACTING_TO_ZERO;
    retractionStartTime = millis();
    
    // Start retraction using large negative target position
    // This will move motors backward slowly for 15 seconds
    Serial.println("Starting retraction to position -5000 at 0.5 mm/s");
    
    drawRetractingToZeroPage();
    return;
  }
}

void handleShutdownConfirmTouch(int x, int y) {
  // YES button - proceed with shutdown
  if (x >= 50 && x <= 210 && y >= 520 && y <= 620) {
    Serial.println("=== SHUTDOWN INITIATED ===");
    
    // Turn off heaters immediately
    heatControlEnabled = false;
    analogWrite(MOSFET_PIN, 0);
    Serial.println("Heaters OFF");
    
    // Go to shutting down page
    currentPage = SHUTTING_DOWN;
    shutdownInProgress = true;
    shutdownStartTime = millis();
    drawShuttingDownPage();
    
    Serial.println("Starting motor retraction to zero...");
    // Start movement to zero
    moveMotorsTo(0, 0, 0.5);  // Slow retraction
    return;
  }
  
  // NO button - cancel shutdown
  if (x >= 270 && x <= 430 && y >= 520 && y <= 620) {
    Serial.println("Shutdown cancelled");
    currentPage = HOME;
    drawHomePage();
    return;
  }
}

void handleWelcomeTouch(int x, int y) {
  if (x >= 140 && x <= 340 && y >= 630 && y <= 730) {
    currentPage = HOME;
    drawHomePage();
  }
}

void handleHomeTouch(int x, int y) {
  if (x >= 40 && x <= 440) {
    if (y >= 120 && y <= 210) {
      currentPage = TEMPERATURE_PAGE;
      tempSelection = selectedTemp;
      drawParameterPage(tempOptions, numTempOptions, tempSelection, "Temperature", "C");
    } else if (y >= 230 && y <= 320) {
      currentPage = VOLUME1;
      tempSelection = selectedVol1;
      drawParameterPage(volOptions, numVolOptions, tempSelection, "Volume 1", "mL");
    } else if (y >= 340 && y <= 430) {
      currentPage = VOLUME2;
      tempSelection = selectedVol2;
      drawParameterPage(volOptions, numVolOptions, tempSelection, "Volume 2", "mL");
    } else if (y >= 450 && y <= 540) {
      currentPage = CONCENTRATION;
      tempSelection = selectedConc;
      drawParameterPage(concOptions, numConcOptions, tempSelection, "Concentration", "%");
    }
  }
  
  // CONTINUE button
  if (x >= 40 && x <= 220 && y >= 600 && y <= 680) {
    if (selectedTemp < 0 || selectedVol1 < 0 || selectedVol2 < 0 || selectedConc < 0) {
      currentPage = ERROR_PAGE;
      drawErrorPage("Please specify all parameters");
    } else {
      currentPage = LOADING_SYRINGES;
      drawLoadingSyringesPage();
    }
  }
  
  // SHUTDOWN button
  if (x >= 260 && x <= 440 && y >= 600 && y <= 680) {
    currentPage = SHUTDOWN_CONFIRM;
    drawShutdownConfirmPage();
  }
}

void handleParameterTouch(int x, int y, int* options, int numOptions, float* targetVar) {
  int boxX = 40;
  int boxY = 75;
  int boxWidth = 260;
  int boxHeight = 100;
  int buttonSize = 70;
  int buttonSpacing = 10;
  
  float step = 1.0f;
  float minVal = 0;
  float maxVal = 100;
  
  if (currentPage == TEMPERATURE_PAGE) {
    step = 1.0f;
    minVal = tempOptions[0];
    maxVal = tempOptions[numTempOptions - 1];
  } else if (currentPage == VOLUME1 || currentPage == VOLUME2) {
    step = 0.1f;
    minVal = 0.1f;
    maxVal = 10.0f;
  } else if (currentPage == CONCENTRATION) {
    step = 5.0f;
    minVal = concOptions[0];
    maxVal = concOptions[numConcOptions - 1];
  } else if (currentPage == SPEED) {
    step = 0.1f;
    minVal = speedOptions[0];
    maxVal = speedOptions[numSpeedOptions - 1];
  }
  
  if (x >= boxX && x <= boxX + buttonSize && y >= boxY && y <= boxY + boxHeight) {
    if (tempSelection >= 0) {
      tempSelection -= step;
      if (tempSelection < minVal) tempSelection = minVal;
      if (step < 1.0f) {
        tempSelection = round(tempSelection * 10.0) / 10.0;
      } else {
        tempSelection = round(tempSelection);
      }
      const char* title;
      const char* unit;
      getCurrentPageInfo(&title, &unit);
      drawParameterPage(options, numOptions, tempSelection, title, unit);
    }
    return;
  }
  
  int plusX = boxX + buttonSize + buttonSpacing + boxWidth + buttonSpacing;
  if (x >= plusX && x <= plusX + buttonSize && y >= boxY && y <= boxY + boxHeight) {
    if (tempSelection >= 0) {
      tempSelection += step;
      if (tempSelection > maxVal) tempSelection = maxVal;
      if (step < 1.0f) {
        tempSelection = round(tempSelection * 10.0) / 10.0;
      } else {
        tempSelection = round(tempSelection);
      }
      const char* title;
      const char* unit;
      getCurrentPageInfo(&title, &unit);
      drawParameterPage(options, numOptions, tempSelection, title, unit);
    } else {
      tempSelection = options[0];
      const char* title;
      const char* unit;
      getCurrentPageInfo(&title, &unit);
      drawParameterPage(options, numOptions, tempSelection, title, unit);
    }
    return;
  }
  
  if (x >= 40 && x <= 160 && y >= 700 && y <= 770) {
    currentPage = HOME;
    drawHomePage();
    return;
  }
  
  if (x >= 180 && x <= 300 && y >= 700 && y <= 770) {
    currentPage = WELCOME;
    drawWelcomePage();
    return;
  }
  
  if (x >= 320 && x <= 440 && y >= 700 && y <= 770) {
    if (currentPage == TEMPERATURE_PAGE && tempSelection > 0) {
      selectedTemp = tempSelection;
      Setpoint_Syringe = selectedTemp;
    } else if (tempSelection >= 0) {
      *targetVar = tempSelection;
    }
    currentPage = HOME;
    drawHomePage();
    return;
  }
  
  int cols = 3;
  int buttonWidth = 120;
  int buttonHeight = 80;
  int spacingX = 20;
  int spacingY = 20;
  int startX = 40;
  int startY = 200;
  
  for (int i = 0; i < numOptions; i++) {
    int col = i % cols;
    int row = i / cols;
    int bx = startX + col * (buttonWidth + spacingX);
    int by = startY + row * (buttonHeight + spacingY);
    
    if (x >= bx && x <= bx + buttonWidth && y >= by && y <= by + buttonHeight) {
      tempSelection = options[i];
      const char* title;
      const char* unit;
      getCurrentPageInfo(&title, &unit);
      drawParameterPage(options, numOptions, tempSelection, title, unit);
      break;
    }
  }
}

void handlePrintConfirmTouch(int x, int y) {
  if (x >= 140 && x <= 340 && y >= 630 && y <= 700) {
    Setpoint_Syringe = selectedTemp;
    heatControlEnabled = true;
    systemReady = false;
    
    currentPage = LOADING_PAGE;
    drawLoadingPage();
    delay(500);
    
    // Skip executeLoad - motors are already positioned from homing!
    current_state = LOAD;  // Just set the state
    
    if (!executeSetup(selectedVol1, selectedVol2, selectedConc)) {
      drawErrorPage("Setup failed");
      currentPage = ERROR_PAGE;
      return;
    }
    drawLoadingPage();
    delay(500);
    
    if (!executePrime()) {
      drawErrorPage("Prime failed");
      currentPage = ERROR_PAGE;
      return;
    }
    drawLoadingPage();
    delay(500);
    
    currentPage = TEMP_READY;
    tempStableTime = 0;
    systemReady = false;
    drawTempReadyPage();
  }
  
  if (x >= 40 && x <= 160 && y >= 720 && y <= 770) {
    currentPage = LOADING_SYRINGES;
    drawLoadingSyringesPage();
  }
  
  if (x >= 320 && x <= 440 && y >= 720 && y <= 770) {
    currentPage = WELCOME;
    drawWelcomePage();
  }
}

void handleTempReadyTouch(int x, int y) {
  if (x >= 90 && x <= 390 && y >= 600 && y <= 680 && systemReady) {
    currentPage = EXTRUSION_SETUP;
    drawExtrusionSetupPage();
  }
  
  if (x >= 140 && x <= 340 && y >= 700 && y <= 760) {
    heatControlEnabled = false;
    analogWrite(MOSFET_PIN, 0);
    currentPage = HOME;
    drawHomePage();
  }
}

void handleExtrusionSetupTouch(int x, int y) {
  // Volume controls
  if (x >= 50 && x <= 130 && y >= 200 && y <= 270) {
    // Decrease volume
    extrusionVolume -= 0.1;
    if (extrusionVolume < 0.1) extrusionVolume = 0.1;
    extrusionVolume = round(extrusionVolume * 10.0) / 10.0;
    drawExtrusionSetupPage();
    return;
  }
  
  if (x >= 350 && x <= 430 && y >= 200 && y <= 270) {
    // Increase volume
    extrusionVolume += 0.1;
    if (extrusionVolume > 20.0) extrusionVolume = 20.0;  // Max 20mL
    extrusionVolume = round(extrusionVolume * 10.0) / 10.0;
    drawExtrusionSetupPage();
    return;
  }
  
  // Print Time controls
  if (x >= 50 && x <= 130 && y >= 330 && y <= 400) {
    // Decrease time
    printTime -= 1.0;
    if (printTime < 2.0) printTime = 2.0;
    drawExtrusionSetupPage();
    return;
  }
  
  if (x >= 350 && x <= 430 && y >= 330 && y <= 400) {
    // Increase time
    printTime += 1.0;
    if (printTime > 15.0) printTime = 15.0;
    drawExtrusionSetupPage();
    return;
  }
  
  // START button
  if (x >= 90 && x <= 390 && y >= 600 && y <= 680) {
    // VALIDATE EXTRUSION BEFORE STARTING
    ExtrusionValidation validation = validateExtrusion(extrusionVolume, printTime);
    
    if (!validation.is_valid) {
      // Show validation error with suggestions
      Serial.println("=== EXTRUSION VALIDATION FAILED ===");
      Serial.println(validation.error_message);
      Serial.println("Suggestions:");
      Serial.println(validation.suggestion);
      
      drawValidationErrorPage(validation.error_message, validation.suggestion);
      // Stay on same page type (will go back on BACK button)
      return;
    }
    
    // Validation passed - proceed to print
    Serial.println("=== EXTRUSION VALIDATED ===");
    currentPage = READY_TO_PRINT;
    drawReadyToPrintPage();
    return;
  }
  
  // HOME button
  if (x >= 140 && x <= 340 && y >= 700 && y <= 760) {
    heatControlEnabled = false;
    analogWrite(MOSFET_PIN, 0);
    currentPage = HOME;
    drawHomePage();
    return;
  }
}

void handlePostExtrusionOptionsTouch(int x, int y) {
  // SAME EXTRUSION button
  if (x >= 40 && x <= 160 && y >= 450 && y <= 520) {
    // Go back to READY_TO_PRINT with same settings
    currentPage = READY_TO_PRINT;
    drawReadyToPrintPage();
    return;
  }
  
  // ADJUST PARAMETERS button
  if (x >= 180 && x <= 300 && y >= 450 && y <= 520) {
    // Go to EXTRUSION_SETUP to change volume/time
    currentPage = EXTRUSION_SETUP;
    drawExtrusionSetupPage();
    return;
  }
  
  // FINISH button
  if (x >= 320 && x <= 440 && y >= 450 && y <= 520) {
    Serial.println("FINISH pressed - returning motors to load position");
    // Move motors to 15000 (LOAD_POSITION)
    if (moveMotorsTo(LOAD_POSITION, LOAD_POSITION, 3.0)) {
      arduino_pos1 = LOAD_POSITION;
      arduino_pos2 = LOAD_POSITION;
      Serial.println("Motors at LOAD_POSITION - returning to start");
      
      // Turn off heat
      heatControlEnabled = false;
      analogWrite(MOSFET_PIN, 0);
      
      // Return to initial screen
      currentPage = MOTOR_ZERO_CHECK;
      drawMotorZeroCheckPage();
    } else {
      drawErrorPage("Failed to return to load position");
      currentPage = ERROR_PAGE;
    }
    return;
  }
  
  // HOME button
  if (x >= 140 && x <= 340 && y >= 600 && y <= 670) {
    heatControlEnabled = false;
    analogWrite(MOSFET_PIN, 0);
    currentPage = HOME;
    drawHomePage();
    return;
  }
}

void handleReadyToPrintTouch(int x, int y) {
  if (x >= 90 && x <= 390 && y >= 500 && y <= 620) {
    // Initialize cycle tracking
    cycleStartDispensed1 = config.dispensed1;
    cycleStartDispensed2 = config.dispensed2;
    cycleTargetVol1 = extrusionVolume * config.ratio1;
    cycleTargetVol2 = extrusionVolume * config.ratio2;
    
    isPrinting = true;
    currentPage = PRINTING_PAGE;
    drawPrintingPage();
  }
  
  if (x >= 140 && x <= 340 && y >= 670 && y <= 740) {
    heatControlEnabled = false;
    analogWrite(MOSFET_PIN, 0);
    currentPage = HOME;
    drawHomePage();
  }
}

void handlePrintingTouch(int x, int y) {
  if (x >= 140 && x <= 340 && y >= 600 && y <= 680) {
    isPrinting = false;
    
    tic1.haltAndHold();
    tic2.haltAndHold();
    delay(100);
    
    long stopped_pos1 = tic1.getCurrentPosition();
    long stopped_pos2 = tic2.getCurrentPosition();
    
    arduino_pos1 = stopped_pos1;
    arduino_pos2 = stopped_pos2;
    
    float actualDispensed1 = (config.prime_pos1 - arduino_pos1) / STEPS_PER_ML;
    float actualDispensed2 = (config.prime_pos2 - arduino_pos2) / STEPS_PER_ML;
    
    config.remaining1 = config.syringe_vol1 - actualDispensed1;
    config.remaining2 = config.syringe_vol2 - actualDispensed2;
    config.dispensed1 = actualDispensed1;
    config.dispensed2 = actualDispensed2;
    
    current_state = COMPLETE;
    currentPage = READY_TO_PRINT;
    drawReadyToPrintPage();
  }
}

void handleLoadingTouch(int x, int y) {
  if (x >= 140 && x <= 340 && y >= 680 && y <= 760) {
    heatControlEnabled = false;
    analogWrite(MOSFET_PIN, 0);
    currentPage = HOME;
    drawHomePage();
  }
}

void handlePrintDoneTouch(int x, int y) {
  // FINISH button (left)
  if (x >= 40 && x <= 220 && y >= 550 && y <= 630) {
    Serial.println("FINISH pressed from PRINT_DONE - returning to start");
    // Move motors to LOAD_POSITION
    if (moveMotorsTo(LOAD_POSITION, LOAD_POSITION, 3.0)) {
      arduino_pos1 = LOAD_POSITION;
      arduino_pos2 = LOAD_POSITION;
      Serial.println("Motors at LOAD_POSITION - returning to start");
      
      // Turn off heat
      heatControlEnabled = false;
      analogWrite(MOSFET_PIN, 0);
      
      // Return to initial screen
      currentPage = MOTOR_ZERO_CHECK;
      drawMotorZeroCheckPage();
    } else {
      drawErrorPage("Failed to return to load position");
      currentPage = ERROR_PAGE;
    }
    return;
  }
  
  // HOME button (right)
  if (x >= 260 && x <= 440 && y >= 550 && y <= 630) {
    heatControlEnabled = false;
    analogWrite(MOSFET_PIN, 0);
    currentPage = HOME;
    drawHomePage();
  }
}

void handleErrorTouch(int x, int y) {
  // Original error page HOME button
  if (x >= 165 && x <= 315 && y >= 450 && y <= 520) {
    heatControlEnabled = false;
    analogWrite(MOSFET_PIN, 0);
    currentPage = HOME;
    drawHomePage();
    return;
  }
  
  // Validation error page BACK button (goes back to extrusion setup)
  if (x >= 140 && x <= 340 && y >= 680 && y <= 750) {
    currentPage = EXTRUSION_SETUP;
    drawExtrusionSetupPage();
    return;
  }
}

void handleLoadingSyringesTouch(int x, int y) {
  if (x >= 90 && x <= 390 && y >= 580 && y <= 660) {
    currentPage = HOMING_PAGE;
    drawHomingPage();
    
    // Small delay to show the homing page
    delay(100);
    
    // Attempt to home motors to LOAD_POSITION (15000)
    if (!homeMotors()) {
      drawErrorPage("Failed to reach loading position!");
      currentPage = ERROR_PAGE;
      return;
    }
    
    // Success - motors are now at LOAD_POSITION (15000)
    currentPage = WAITING_FOR_SYRINGES;
    drawWaitingForSyringesPage();
    
    return;
  }
  
  if (x >= 40 && x <= 160 && y >= 700 && y <= 770) {
    currentPage = HOME;
    drawHomePage();
  }
  
  if (x >= 320 && x <= 440 && y >= 700 && y <= 770) {
    currentPage = WELCOME;
    drawWelcomePage();
  }
}

void handleWaitingForSyringesTouch(int x, int y) {
  if (x >= 90 && x <= 390 && y >= 600 && y <= 680) {
    currentPage = HOMING_PAGE;
    drawHomingPage();
    
    // Small delay to show the homing page
    delay(100);
    
    // Complete the homing process - move to volume position
    if (completeHoming()) {
      // Success - go to print confirm
      currentPage = PRINT_CONFIRM;
      drawPrintConfirmPage();
    } else {
      // Failed
      drawErrorPage("Homing completion failed!");
      currentPage = ERROR_PAGE;
    }
    return;
  }
  
  if (x >= 170 && x <= 310 && y >= 710 && y <= 770) {
    currentPage = WELCOME;
    drawWelcomePage();
  }
}

void handleTouch(int x, int y) {
  switch (currentPage) {
    case MOTOR_ZERO_CHECK:
      handleMotorZeroCheckTouch(x, y);
      break;
    case RETRACTING_TO_ZERO:
      // No touch interaction during retraction
      break;
    case CALIBRATION_IN_PROGRESS:
      // No touch interaction during calibration
      break;
    case WELCOME:
      handleWelcomeTouch(x, y);
      break;
    case HOME:
      handleHomeTouch(x, y);
      break;
    case TEMPERATURE_PAGE:
      handleParameterTouch(x, y, tempOptions, numTempOptions, &selectedTemp);
      break;
    case VOLUME1:
      handleParameterTouch(x, y, volOptions, numVolOptions, &selectedVol1);
      break;
    case VOLUME2:
      handleParameterTouch(x, y, volOptions, numVolOptions, &selectedVol2);
      break;
    case CONCENTRATION:
      handleParameterTouch(x, y, concOptions, numConcOptions, &selectedConc);
      break;
    case SPEED:
      handleParameterTouch(x, y, speedOptions, numSpeedOptions, &selectedSpeed);
      break;
    case PRINT_CONFIRM:
      handlePrintConfirmTouch(x, y);
      break;
    case LOADING_PAGE:
      handleLoadingTouch(x, y);
      break;
    case LOADING_SYRINGES:
      handleLoadingSyringesTouch(x, y);
      break;
    case WAITING_FOR_SYRINGES:
      handleWaitingForSyringesTouch(x, y);
      break;
    case TEMP_READY:
      handleTempReadyTouch(x, y);
      break;
    case EXTRUSION_SETUP:
      handleExtrusionSetupTouch(x, y);
      break;
    case READY_TO_PRINT:
      handleReadyToPrintTouch(x, y);
      break;
    case PRINTING_PAGE:
      handlePrintingTouch(x, y);
      break;
    case POST_EXTRUSION_OPTIONS:
      handlePostExtrusionOptionsTouch(x, y);
      break;
    case PRINT_DONE:
      handlePrintDoneTouch(x, y);
      break;
    case SHUTDOWN_CONFIRM:
      handleShutdownConfirmTouch(x, y);
      break;
    case SHUTTING_DOWN:
      // No touch interaction during shutdown
      break;
    case SHUTDOWN_COMPLETE:
      // No touch interaction - system is shut down
      break;
    case ERROR_PAGE:
      handleErrorTouch(x, y);
      break;
  }
}

void getCurrentPageInfo(const char** title, const char** unit) {
  if (currentPage == TEMPERATURE_PAGE) {
    *title = "Temperature";
    *unit = "C";
  } else if (currentPage == VOLUME1) {
    *title = "Volume 1";
    *unit = "mL";
  } else if (currentPage == VOLUME2) {
    *title = "Volume 2";
    *unit = "mL";
  } else if (currentPage == CONCENTRATION) {
    *title = "Concentration";
    *unit = "%";
  } else if (currentPage == SPEED) {
    *title = "Speed";
    *unit = "mm/s";
  }
}

// ==================== CONTINUOUS PRINTING ====================
void performContinuousPrint() {
  // This function should only execute ONE extrusion cycle when called
  // It gets called repeatedly in the loop when isPrinting is true
  // So we need to stop after one cycle
  
  // Execute ONE extrusion cycle with current settings
  if (executeExtrude(extrusionVolume, printTime)) {
    // SUCCESS - stop printing and go to options page
    isPrinting = false;
    currentPage = POST_EXTRUSION_OPTIONS;
    drawPostExtrusionOptionsPage();
  } else {
    // FAILED - check why
    isPrinting = false;
    if (config.remaining1 < 0.1 || config.remaining2 < 0.1) {
      // Syringes are empty
      drawPrintDonePage();
      currentPage = PRINT_DONE;
    } else {
      // Insufficient volume - send to EXTRUSION_SETUP to adjust
      Serial.println("Insufficient volume - returning to EXTRUSION_SETUP");
      currentPage = EXTRUSION_SETUP;
      drawExtrusionSetupPage();
    }
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  delay(1000);
  
  Serial.println("\n================================");
  Serial.println("  BIOPRINT AM - INTEGRATED v3.1");
  Serial.println("  + IMPROVED MOTOR CONTROL");
  Serial.println("  + PID TEMPERATURE CONTROL");
  Serial.println("  + FIXED COORDINATE SYSTEM");
  Serial.println("================================");
  
  analogReadResolution(12);
  
  pinMode(MOSFET_PIN, OUTPUT);
  analogWrite(MOSFET_PIN, 0);
  
  display.begin();
  touchDetector.begin();
  display.setRotation(0);
  display.fillScreen(BG_COLOR);
  
  initializeMotors();
  // Don't calibrate here - wait for user confirmation
  updateTemperatures();
  
  Serial.println("System ready!");
  Serial.print("LOAD_POSITION = ");
  Serial.println(LOAD_POSITION);
  Serial.println("Position reference: 15000 = load, 11600 = 10mL, 1600 = 0mL");
  Serial.println("Waiting for user to confirm motor zero position...");
  
  currentPage = MOTOR_ZERO_CHECK;
  drawMotorZeroCheckPage();
}

// ==================== MAIN LOOP ====================
void loop() {
  // Handle retraction timer
  if (currentPage == RETRACTING_TO_ZERO) {
    unsigned long elapsed = millis() - retractionStartTime;
    
    // Start the movement on first loop iteration
    static bool retractionStarted = false;
    if (!retractionStarted) {
      // Move to large negative position at 0.5 mm/s
      // This will take about 15 seconds to move -5000 steps
      moveMotorsTo(-5000, -5000, 0.5);
      retractionStarted = true;
    }
    
    // Update display every 200ms
    static unsigned long lastRetractionUpdate = 0;
    if (millis() - lastRetractionUpdate > 200) {
      drawRetractingToZeroPage();
      lastRetractionUpdate = millis();
    }
    
    // Check if retraction complete (15 seconds elapsed)
    if (elapsed >= RETRACTION_DURATION) {
      Serial.println("Retraction time complete - stopping motors");
      retractionStarted = false;  // Reset for next time
      
      // Stop motors and halt
      tic1.haltAndHold();
      tic2.haltAndHold();
      delay(100);
      
      // Set current position as zero
      tic1.haltAndSetPosition(0);
      tic2.haltAndSetPosition(0);
      delay(50);
      
      // Re-enable motors after position reset
      tic1.exitSafeStart();
      tic2.exitSafeStart();
      
      arduino_pos1 = 0;
      arduino_pos2 = 0;
      
      Serial.println("Motors halted, position set to 0, and re-enabled");
      
      // Now run calibration
      currentPage = CALIBRATION_IN_PROGRESS;
      drawCalibrationInProgressPage();
      delay(500);
      
      calibrateMotorsOnStartup();
      calibrationComplete = true;
      
      currentPage = WELCOME;
      drawWelcomePage();
    }
    
    return;  // Skip rest of loop during retraction
  }
  
  // Handle shutdown sequence
  if (currentPage == SHUTTING_DOWN && shutdownInProgress) {
    // Update display every 500ms
    static unsigned long lastShutdownUpdate = 0;
    if (millis() - lastShutdownUpdate > 500) {
      drawShuttingDownPage();
      lastShutdownUpdate = millis();
    }
    
    // Check if motors have reached zero
    if (arduino_pos1 <= 10 && arduino_pos2 <= 10) {
      Serial.println("=== SHUTDOWN COMPLETE ===");
      Serial.println("Motors at zero position");
      
      // Deenergize motors
      tic1.deenergize();
      tic2.deenergize();
      
      shutdownInProgress = false;
      currentPage = SHUTDOWN_COMPLETE;
      drawShutdownCompletePage();
      
      Serial.println("System safe to power off");
    }
    
    return;  // Skip rest of loop during shutdown
  }
  
  if (millis() - lastTempUpdate > TEMP_UPDATE_INTERVAL) {
    updateTemperatures();
    lastTempUpdate = millis();
    
    if (currentPage == LOADING_PAGE) {
      drawLoadingPage();
    }
    
    if (currentPage == TEMP_READY) {
      bool tempStable = (currentDisplayTemp > -999.0) && 
                        (abs(currentDisplayTemp - Setpoint_Syringe) <= TEMP_TOLERANCE);
      bool actuatorsReady = (current_state == PRIMED) && motorsHomed;
      
      if (tempStable && actuatorsReady) {
        if (tempStableTime == 0) {
          tempStableTime = millis();
        } else if (millis() - tempStableTime >= TEMP_STABLE_DURATION) {
          if (!systemReady) {
            systemReady = true;
            drawTempReadyPage();
          }
        }
      } else {
        tempStableTime = 0;
        if (systemReady) {
          systemReady = false;
          drawTempReadyPage();
        }
      }
      
      static unsigned long lastTempReadyUpdate = 0;
      if (millis() - lastTempReadyUpdate > 2000) {
        drawTempReadyPage();
        lastTempReadyUpdate = millis();
      }
    }
  }
  
  applyHeatControl();
  
  if (isPrinting && currentPage == PRINTING_PAGE) {
    performContinuousPrint();
  }
  
  uint8_t contacts;
  GDTpoint_t points[5];
  contacts = touchDetector.getTouchPoints(points);
  
  if (contacts > 0 && !lastTouchState) {
    int x = points[0].x;
    int y = points[0].y;
    
    handleTouch(x, y);
    lastTouchState = true;
    delay(250);
  } else if (contacts == 0) {
    lastTouchState = false;
  }
  
  if (current_state == EXTRUDING) {
    tic1.resetCommandTimeout();
    tic2.resetCommandTimeout();
  }
  
  delay(10);
}
