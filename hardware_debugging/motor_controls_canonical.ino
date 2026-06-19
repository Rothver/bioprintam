/*
 * Arduino GIGA R1 WiFi - Dual Syringe Extruder Control (Canonical Version)
 * I2C control of two Pololu TIC T825 stepper drivers (IDs 14 & 15)
 * 
 * CANONICAL CONSOLIDATED VERSION - Phase 7 Refactoring
 * Uses unified libraries for motor control, thermistor reading, and state management
 * 
 * Hardware:
 * - Arduino GIGA R1 WiFi
 * - Motors: Actuonix P8-ST-100mm, 1009 steps/mL, 0.006 mm/step
 * - I2C communication via Wire (SCL/SDA pins)
 * 
 * Position Convention:
 * - 0 steps = Fully retracted (no extrusion, empty syringe position)
 * - Max steps = Fully extended (all material dispensed)
 * - Device homes to 0 on startup and shutdown for easy reloading
 * 
 * REFACTORING NOTES:
 * - Motor configuration moved to config/config.h
 * - Motor helper functions use libraries/motor_controller.h
 * - State machine moved to libraries/state_machine.h
 * - All calibration constants centralized in config/config.h
 * - No code duplication between this and integrated firmware
 */

#include <Tic.h>
#include <Wire.h>
#include "config/config.h"
#include "libraries/motor_controller.h"
#include "libraries/state_machine.h"

// ==================== MOTOR CONFIGURATION ====================
// Motor address and constant configuration moved to config/config.h
// Motor conversion helpers: mlToSteps(), stepsToMl(), mmsToStepsPerSec(), stepsPerSecToTicUnits()
// are defined in libraries/motor_controller.h

// Motor objects (I2C)
TicI2C tic1;
TicI2C tic2;

// ==================== SYSTEM STATE ====================
struct SystemState {
  float ratio1 = 1.0f;
  float ratio2 = 1.0f;
  float initial_vol1 = 10.0f;
  float initial_vol2 = 10.0f;
  
  long current_pos1 = 0;
  long current_pos2 = 0;
  long target_pos1 = 0;
  long target_pos2 = 0;
  
  float dispensed1_ml = 0.0f;
  float dispensed2_ml = 0.0f;
  float remaining1_ml = 10.0f;
  float remaining2_ml = 10.0f;
  
  bool homed = false;
  bool primed = false;
  bool extruding = false;
  bool error_state = false;
  
  long prime_pos1 = 0;
  long prime_pos2 = 0;
} state;

// ==================== FUNCTION DECLARATIONS ====================
void initializeMotors();
void homeMotors();
void primeMotors();
void startExtrusion(float desired_v1_ml, float desired_speed1_mms);
void checkForErrors();
void printTicError(uint16_t error, int motor_num);
void emergencyHalt();
void printStatus();
void terminateSession();
void printHelp();
void processCommand(String cmd);

// ==================== MOTOR CONTROL ====================
// NOTE: Core motor control functions are INLINE (not abstracted to libraries)
// because they implement CLI-specific behavior and serial debugging
// that is NOT reused in the integrated firmware.
// Motor helper functions are delegated to libraries/motor_controller.h

void initializeMotors() {
  Wire.begin();
  delay(100);
  
  Serial.println("Initializing motors via I2C...");
  
  tic1.setAddress(MOTOR1_ADDRESS);
  delay(10);
  tic1.clearDriverError();
  tic1.haltAndSetPosition(0);
  delay(10);
  tic1.energize();
  delay(10);
  tic1.exitSafeStart();
  delay(10);
  tic1.setMaxAccel(MAX_ACCEL * 100L);
  tic1.setMaxDecel(MAX_ACCEL * 100L);
  
  Serial.print("  Motor 1 initialized at address ");
  Serial.println(MOTOR1_ADDRESS);
  
  tic2.setAddress(MOTOR2_ADDRESS);
  delay(10);
  tic2.clearDriverError();
  tic2.haltAndSetPosition(0);
  delay(10);
  tic2.energize();
  delay(10);
  tic2.exitSafeStart();
  delay(10);
  tic2.setMaxAccel(MAX_ACCEL * 100L);
  tic2.setMaxDecel(MAX_ACCEL * 100L);
  
  Serial.print("  Motor 2 initialized at address ");
  Serial.println(MOTOR2_ADDRESS);
  
  delay(100);
  
  uint16_t err1 = tic1.getErrorStatus();
  uint16_t err2 = tic2.getErrorStatus();
  
  if (err1 != 0 || err2 != 0) {
    Serial.println("✗ Warning: Errors detected after initialization");
    if (err1 != 0) printTicError(err1, 1);
    if (err2 != 0) printTicError(err2, 2);
  } else {
    Serial.println("✓ Motors initialized successfully");
  }
}

void homeMotors() {
  Serial.println("\n=== HOMING ===");
  Serial.println("Moving to position 0 (fully retracted)...");
  
  tic1.haltAndSetPosition(0);
  tic2.haltAndSetPosition(0);
  
  delay(200);
  
  state.current_pos1 = 0;
  state.current_pos2 = 0;
  state.dispensed1_ml = 0.0f;
  state.dispensed2_ml = 0.0f;
  state.remaining1_ml = state.initial_vol1;
  state.remaining2_ml = state.initial_vol2;
  state.primed = false;
  state.extruding = false;
  state.homed = true;
  state.error_state = false;
  
  Serial.println("✓ Homed to 0. Motors fully retracted.");
  Serial.println("✓ Ready for setup and priming.\n");
}

void primeMotors() {
  if (!state.homed) {
    Serial.println("✗ ERROR: Please home motors first!");
    return;
  }
  
  if (state.initial_vol1 <= 0 || state.initial_vol1 > MAX_VOLUME_ML ||
      state.initial_vol2 <= 0 || state.initial_vol2 > MAX_VOLUME_ML) {
    Serial.println("✗ ERROR: Invalid initial volumes!");
    return;
  }
  
  Serial.println("\n=== PRIMING ===");
  
  float missing_vol1 = MAX_VOLUME_ML - state.initial_vol1;
  float missing_vol2 = MAX_VOLUME_ML - state.initial_vol2;
  
  state.prime_pos1 = mlToSteps(missing_vol1);
  state.prime_pos2 = mlToSteps(missing_vol2);
  
  if (state.prime_pos1 > MAX_STEPS || state.prime_pos2 > MAX_STEPS) {
    Serial.println("✗ ERROR: Calculated position exceeds maximum!");
    return;
  }
  
  long current1 = tic1.getCurrentPosition();
  long current2 = tic2.getCurrentPosition();
  
  if (abs(current1 - state.prime_pos1) <= POSITION_TOLERANCE &&
      abs(current2 - state.prime_pos2) <= POSITION_TOLERANCE) {
    Serial.println("Already at prime position.");
    state.current_pos1 = state.prime_pos1;
    state.current_pos2 = state.prime_pos2;
    state.primed = true;
    Serial.println("✓ Ready for extrusion.\n");
    return;
  }
  
  Serial.print("Motor 1: ");
  Serial.print(state.initial_vol1, 2);
  Serial.print(" mL → Position ");
  Serial.print(state.prime_pos1);
  Serial.print(" (");
  Serial.print(missing_vol1, 2);
  Serial.println(" mL offset)");
  
  Serial.print("Motor 2: ");
  Serial.print(state.initial_vol2, 2);
  Serial.print(" mL → Position ");
  Serial.print(state.prime_pos2);
  Serial.print(" (");
  Serial.print(missing_vol2, 2);
  Serial.println(" mL offset)");
  
  long prime_speed = mmsToStepsPerSec(20.0f);
  tic1.setMaxSpeed(stepsPerSecToTicUnits(prime_speed));
  tic2.setMaxSpeed(stepsPerSecToTicUnits(prime_speed));
  
  tic1.setTargetPosition(state.prime_pos1);
  tic2.setTargetPosition(state.prime_pos2);
  
  Serial.print("Moving to prime positions");
  while (abs(tic1.getCurrentPosition() - state.prime_pos1) > POSITION_TOLERANCE ||
         abs(tic2.getCurrentPosition() - state.prime_pos2) > POSITION_TOLERANCE) {
    Serial.print(".");
    tic1.resetCommandTimeout();
    tic2.resetCommandTimeout();
    delay(100);
    checkForErrors();
    if (state.error_state) return;
  }
  
  Serial.println();
  Serial.println("✓ Prime positions reached!");
  
  state.current_pos1 = state.prime_pos1;
  state.current_pos2 = state.prime_pos2;
  
  Serial.println("\nPurging chambers (0.11mL from each motor)...");
  
  const float PURGE_VOLUME = 0.11f;
  long purge_steps = mlToSteps(PURGE_VOLUME);
  
  long purge_target1 = state.current_pos1 + purge_steps;
  long purge_target2 = state.current_pos2 + purge_steps;
  
  if (purge_target1 > MAX_STEPS || purge_target2 > MAX_STEPS ||
      PURGE_VOLUME > state.initial_vol1 || PURGE_VOLUME > state.initial_vol2) {
    Serial.println("✗ WARNING: Not enough volume for purge. Skipping.");
  } else {
    long purge_speed = mmsToStepsPerSec(10.0f);
    tic1.setMaxSpeed(stepsPerSecToTicUnits(purge_speed));
    tic2.setMaxSpeed(stepsPerSecToTicUnits(purge_speed));
    
    tic1.setTargetPosition(purge_target1);
    tic2.setTargetPosition(purge_target2);
    
    Serial.print("Purging");
    while (abs(tic1.getCurrentPosition() - purge_target1) > POSITION_TOLERANCE ||
           abs(tic2.getCurrentPosition() - purge_target2) > POSITION_TOLERANCE) {
      Serial.print(".");
      tic1.resetCommandTimeout();
      tic2.resetCommandTimeout();
      delay(100);
      checkForErrors();
      if (state.error_state) return;
    }
    
    Serial.println();
    Serial.println("✓ Purge complete!");
    
    state.current_pos1 = purge_target1;
    state.current_pos2 = purge_target2;
    state.prime_pos1 = purge_target1;
    state.prime_pos2 = purge_target2;
  }
  
  state.primed = true;
  state.error_state = false;
  
  Serial.println("✓ Priming complete!");
  Serial.println("✓ Ready for extrusion.\n");
}

void startExtrusion(float desired_v1_ml, float desired_speed1_mms) {
  if (!state.primed) {
    Serial.println("✗ ERROR: Please prime motors first!");
    return;
  }
  
  if (state.extruding) {
    Serial.println("✗ ERROR: Already extruding!");
    return;
  }
  
  Serial.println("\n=== EXTRUSION ===");
  
  float v2_ml = desired_v1_ml * (state.ratio2 / state.ratio1);
  
  Serial.print("Ratio: ");
  Serial.print(state.ratio1, 2);
  Serial.print(":");
  Serial.println(state.ratio2, 2);
  
  Serial.print("V1: ");
  Serial.print(desired_v1_ml, 2);
  Serial.print(" mL → V2: ");
  Serial.print(v2_ml, 2);
  Serial.println(" mL");
  
  if (desired_v1_ml <= 0.01f || desired_v1_ml > state.remaining1_ml) {
    Serial.print("✗ ERROR: V1 invalid! (Available: ");
    Serial.print(state.remaining1_ml, 2);
    Serial.println(" mL)");
    return;
  }
  
  if (v2_ml <= 0.01f || v2_ml > state.remaining2_ml) {
    Serial.print("✗ ERROR: V2 exceeds remaining! (Available: ");
    Serial.print(state.remaining2_ml, 2);
    Serial.println(" mL)");
    return;
  }
  
  if (desired_speed1_mms < 0.1f || desired_speed1_mms > MAX_SPEED_MMS) {
    Serial.print("✗ ERROR: Speed out of range! (0.1-");
    Serial.print(MAX_SPEED_MMS, 0);
    Serial.println(" mm/s)");
    return;
  }
  
  state.target_pos1 = state.current_pos1 + mlToSteps(desired_v1_ml);
  state.target_pos2 = state.current_pos2 + mlToSteps(v2_ml);
  
  if (state.target_pos1 > MAX_STEPS || state.target_pos2 > MAX_STEPS) {
    Serial.println("✗ ERROR: Would exceed maximum!");
    return;
  }
  
  float dist1_mm = desired_v1_ml * MM_PER_ML;
  float dist2_mm = v2_ml * MM_PER_ML;
  float max_dist = max(dist1_mm, dist2_mm);
  float extrusion_time = max_dist / desired_speed1_mms;
  
  float speed1_mms = dist1_mm / extrusion_time;
  float speed2_mms = dist2_mm / extrusion_time;
  
  long speed1_steps = mmsToStepsPerSec(speed1_mms);
  long speed2_steps = mmsToStepsPerSec(speed2_mms);
  
  Serial.print("M1: ");
  Serial.print(speed1_mms, 2);
  Serial.print(" mm/s | M2: ");
  Serial.print(speed2_mms, 2);
  Serial.print(" mm/s | Time: ");
  Serial.print(extrusion_time, 2);
  Serial.println(" sec");
  
  tic1.clearDriverError();
  tic2.clearDriverError();
  
  tic1.setMaxSpeed(stepsPerSecToTicUnits(speed1_steps));
  tic2.setMaxSpeed(stepsPerSecToTicUnits(speed2_steps));
  
  tic1.setTargetPosition(state.target_pos1);
  tic2.setTargetPosition(state.target_pos2);
  
  state.extruding = true;
  
  Serial.print("Extruding");
  
  unsigned long last_update = 0;
  while (state.extruding) {
    tic1.resetCommandTimeout();
    tic2.resetCommandTimeout();
    
    long pos1 = tic1.getCurrentPosition();
    long pos2 = tic2.getCurrentPosition();
    
    checkForErrors();
    if (state.error_state) {
      state.current_pos1 = pos1;
      state.current_pos2 = pos2;
      return;
    }
    
    if (pos1 > state.target_pos1 + POSITION_TOLERANCE ||
        pos2 > state.target_pos2 + POSITION_TOLERANCE) {
      Serial.println("\n✗ ERROR: Position overrun!");
      state.current_pos1 = pos1;
      state.current_pos2 = pos2;
      emergencyHalt();
      return;
    }
    
    bool m1_done = abs(pos1 - state.target_pos1) <= POSITION_TOLERANCE;
    bool m2_done = abs(pos2 - state.target_pos2) <= POSITION_TOLERANCE;
    
    if (m1_done && m2_done) {
      state.extruding = false;
      Serial.println();
      Serial.println("✓ Extrusion complete!");
      
      state.current_pos1 = state.target_pos1;
      state.current_pos2 = state.target_pos2;
      
      state.dispensed1_ml = stepsToMl(state.current_pos1 - state.prime_pos1);
      state.dispensed2_ml = stepsToMl(state.current_pos2 - state.prime_pos2);
      state.remaining1_ml = state.initial_vol1 - state.dispensed1_ml;
      state.remaining2_ml = state.initial_vol2 - state.dispensed2_ml;
      
      printStatus();
      break;
    }
    
    if (millis() - last_update > 500) {
      Serial.print(".");
      last_update = millis();
    }
    
    delay(50);
  }
}

void printTicError(uint16_t error, int motor_num) {
  Serial.print("   Motor ");
  Serial.print(motor_num);
  Serial.print(" Error: 0x");
  Serial.print(error, HEX);
  Serial.print(" - ");
  
  if (error & 0x0001) Serial.print("Deenergized ");
  if (error & 0x0002) Serial.print("MotorDriverError ");
  if (error & 0x0004) Serial.print("LowVin ");
  if (error & 0x0008) Serial.print("KillSwitch ");
  if (error & 0x0010) Serial.print("RequiredInputInvalid ");
  if (error & 0x0020) Serial.print("SerialError ");
  if (error & 0x0040) Serial.print("CommandTimeout ");
  if (error & 0x0080) Serial.print("SafeStartViolation ");
  if (error & 0x0100) Serial.print("ERRLine ");
  
  Serial.println();
}

void checkForErrors() {
  uint16_t err1 = tic1.getErrorStatus();
  uint16_t err2 = tic2.getErrorStatus();
  
  if (err1 != 0 || err2 != 0) {
    Serial.println();
    Serial.println("✗ TIC Error Detected!");
    if (err1 != 0) printTicError(err1, 1);
    if (err2 != 0) printTicError(err2, 2);
    
    Serial.println("Attempting to clear...");
    tic1.clearDriverError();
    tic2.clearDriverError();
    tic1.energize();
    tic2.energize();
    tic1.exitSafeStart();
    tic2.exitSafeStart();
    
    delay(100);
    
    err1 = tic1.getErrorStatus();
    err2 = tic2.getErrorStatus();
    
    if (err1 != 0 || err2 != 0) {
      Serial.println("✗ Could not clear - halting!");
      emergencyHalt();
    } else {
      Serial.println("✓ Errors cleared");
    }
  }
}

void emergencyHalt() {
  Serial.println("\n!!! EMERGENCY HALT !!!");
  
  tic1.haltAndHold();
  tic2.haltAndHold();
  
  state.extruding = false;
  state.error_state = true;
  
  Serial.println("Motors halted.");
  Serial.println("Use HOME to reset.\n");
}

void printStatus() {
  Serial.println("\n=== STATUS ===");
  
  state.current_pos1 = tic1.getCurrentPosition();
  state.current_pos2 = tic2.getCurrentPosition();
  
  state.dispensed1_ml = stepsToMl(state.current_pos1 - state.prime_pos1);
  state.dispensed2_ml = stepsToMl(state.current_pos2 - state.prime_pos2);
  state.remaining1_ml = state.initial_vol1 - state.dispensed1_ml;
  state.remaining2_ml = state.initial_vol2 - state.dispensed2_ml;
  
  Serial.println("State:");
  Serial.print("  Homed: ");
  Serial.println(state.homed ? "YES" : "NO");
  Serial.print("  Primed: ");
  Serial.println(state.primed ? "YES" : "NO");
  Serial.print("  Extruding: ");
  Serial.println(state.extruding ? "YES" : "NO");
  
  Serial.println("\nSetup:");
  Serial.print("  Ratio: ");
  Serial.print(state.ratio1, 2);
  Serial.print(":");
  Serial.println(state.ratio2, 2);
  Serial.print("  Initial: ");
  Serial.print(state.initial_vol1, 2);
  Serial.print(" / ");
  Serial.print(state.initial_vol2, 2);
  Serial.println(" mL");
  
  Serial.println("\nMotor 1:");
  Serial.print("  Position: ");
  Serial.print(state.current_pos1);
  Serial.print(" steps (");
  Serial.print(state.current_pos1 * MM_PER_STEP, 2);
  Serial.println(" mm)");
  Serial.print("  Dispensed: ");
  Serial.print(state.dispensed1_ml, 2);
  Serial.print(" / Remaining: ");
  Serial.print(state.remaining1_ml, 2);
  Serial.println(" mL");
  
  Serial.println("\nMotor 2:");
  Serial.print("  Position: ");
  Serial.print(state.current_pos2);
  Serial.print(" steps (");
  Serial.print(state.current_pos2 * MM_PER_STEP, 2);
  Serial.println(" mm)");
  Serial.print("  Dispensed: ");
  Serial.print(state.dispensed2_ml, 2);
  Serial.print(" / Remaining: ");
  Serial.print(state.remaining2_ml, 2);
  Serial.println(" mL");
  Serial.println();
}

void terminateSession() {
  if (state.extruding) {
    Serial.println("✗ ERROR: Cannot terminate during extrusion!");
    return;
  }
  
  Serial.println("\n=== TERMINATING SESSION ===");
  
  long current_pos1 = tic1.getCurrentPosition();
  long current_pos2 = tic2.getCurrentPosition();
  
  Serial.print("Reading from TIC boards - M1: ");
  Serial.print(current_pos1);
  Serial.print(" steps, M2: ");
  Serial.print(current_pos2);
  Serial.println(" steps");
  
  if (abs(current_pos1) <= POSITION_TOLERANCE && abs(current_pos2) <= POSITION_TOLERANCE) {
    Serial.println("Already at home position (0).");
    state.homed = false;
    state.primed = false;
    Serial.println("✓ Session terminated.\n");
    return;
  }
  
  uint16_t err1 = tic1.getErrorStatus();
  uint16_t err2 = tic2.getErrorStatus();
  if (err1 != 0 || err2 != 0) {
    Serial.println("TIC errors detected before retraction:");
    if (err1 != 0) printTicError(err1, 1);
    if (err2 != 0) printTicError(err2, 2);
    Serial.println("Attempting to clear errors...");
    tic1.clearDriverError();
    tic2.clearDriverError();
    tic1.energize();
    tic2.energize();
    tic1.exitSafeStart();
    tic2.exitSafeStart();
    delay(100);
  }
  
  Serial.println("Setting retraction parameters...");
  
  long retract_speed = mmsToStepsPerSec(20.0f);
  Serial.print("Retract speed: ");
  Serial.print(retract_speed);
  Serial.println(" steps/sec");
  
  tic1.setMaxSpeed(stepsPerSecToTicUnits(retract_speed));
  tic2.setMaxSpeed(stepsPerSecToTicUnits(retract_speed));
  
  tic1.clearDriverError();
  tic2.clearDriverError();
  tic1.energize();
  tic2.energize();
  tic1.exitSafeStart();
  tic2.exitSafeStart();
  
  delay(50);
  
  Serial.println("Commanding motors to position 0...");
  tic1.setTargetPosition(0);
  tic2.setTargetPosition(0);
  
  delay(100);
  
  Serial.print("Target positions set - M1: ");
  Serial.print(tic1.getTargetPosition());
  Serial.print(", M2: ");
  Serial.println(tic2.getTargetPosition());
  
  Serial.println("Retracting (press STOP to abort)...");
  
  unsigned long last_update = 0;
  unsigned long start_time = millis();
  int loop_count = 0;
  
  while (true) {
    loop_count++;
    
    tic1.resetCommandTimeout();
    tic2.resetCommandTimeout();
    
    long pos1 = tic1.getCurrentPosition();
    long pos2 = tic2.getCurrentPosition();
    
    err1 = tic1.getErrorStatus();
    err2 = tic2.getErrorStatus();
    if (err1 != 0 || err2 != 0) {
      Serial.println("\n✗ Error during retraction!");
      if (err1 != 0) printTicError(err1, 1);
      if (err2 != 0) printTicError(err2, 2);
      emergencyHalt();
      return;
    }
    
    if (abs(pos1) <= POSITION_TOLERANCE && abs(pos2) <= POSITION_TOLERANCE) {
      Serial.println("\n✓ Both motors reached position 0!");
      break;
    }
    
    if (millis() - start_time > 30000) {
      Serial.println("\n✗ Timeout! Motors did not reach position 0.");
      Serial.print("Final positions - M1: ");
      Serial.print(pos1);
      Serial.print(", M2: ");
      Serial.println(pos2);
      emergencyHalt();
      return;
    }
    
    if (millis() - last_update > 2000) {
      Serial.print("Loop ");
      Serial.print(loop_count);
      Serial.print(" | M1: ");
      Serial.print(pos1);
      Serial.print(" steps, M2: ");
      Serial.print(pos2);
      Serial.print(" steps | Time: ");
      Serial.print((millis() - start_time) / 1000);
      Serial.println("s");
      last_update = millis();
    }
    
    delay(50);
  }
  
  tic1.haltAndSetPosition(0);
  tic2.haltAndSetPosition(0);
  
  Serial.println("✓ Motors retracted to home position.");
  Serial.println("✓ Session terminated. Safe to reload syringes.");
  Serial.println("✓ Use HOME to begin new session.\n");
  
  state.homed = false;
  state.primed = false;
  state.current_pos1 = 0;
  state.current_pos2 = 0;
  state.dispensed1_ml = 0.0f;
  state.dispensed2_ml = 0.0f;
}

void printHelp() {
  Serial.println("\n=== COMMANDS ===");
  Serial.println("HOME - Reset to position 0");
  Serial.println("SETUP <r1> <r2> <v1> <v2> - Set ratio & volumes");
  Serial.println("PRIME - Move to start positions");
  Serial.println("EXTRUDE <vol1> <speed> - Dispense material");
  Serial.println("STATUS - Show positions & volumes");
  Serial.println("TERMINATE - Return to home");
  Serial.println("STOP - Emergency halt");
  Serial.println("HELP - Show this message\n");
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  if (cmd.startsWith("HOME")) {
    homeMotors();
  }
  else if (cmd.startsWith("SETUP")) {
    int idx1 = cmd.indexOf(' ');
    int idx2 = cmd.indexOf(' ', idx1 + 1);
    int idx3 = cmd.indexOf(' ', idx2 + 1);
    int idx4 = cmd.indexOf(' ', idx3 + 1);
    
    if (idx1 > 0 && idx2 > 0 && idx3 > 0 && idx4 > 0) {
      state.ratio1 = cmd.substring(idx1 + 1, idx2).toFloat();
      state.ratio2 = cmd.substring(idx2 + 1, idx3).toFloat();
      state.initial_vol1 = cmd.substring(idx3 + 1, idx4).toFloat();
      state.initial_vol2 = cmd.substring(idx4 + 1).toFloat();
      
      if (state.ratio1 <= 0 || state.ratio2 <= 0) {
        Serial.println("✗ ERROR: Ratio must be > 0");
        return;
      }
      
      if (state.initial_vol1 <= 0 || state.initial_vol1 > MAX_VOLUME_ML ||
          state.initial_vol2 <= 0 || state.initial_vol2 > MAX_VOLUME_ML) {
        Serial.println("✗ ERROR: Volumes must be 0.1-10 mL");
        return;
      }
      
      state.remaining1_ml = state.initial_vol1;
      state.remaining2_ml = state.initial_vol2;
      
      Serial.println("\n✓ Setup updated");
      Serial.print("  Ratio: ");
      Serial.print(state.ratio1, 2);
      Serial.print(":");
      Serial.println(state.ratio2, 2);
      Serial.print("  Volumes: ");
      Serial.print(state.initial_vol1, 2);
      Serial.print(" / ");
      Serial.print(state.initial_vol2, 2);
      Serial.println(" mL");
      Serial.println("→ Use PRIME next\n");
    } else {
      Serial.println("✗ Format: SETUP <r1> <r2> <v1> <v2>");
    }
  }
  else if (cmd.startsWith("PRIME")) {
    primeMotors();
  }
  else if (cmd.startsWith("EXTRUDE")) {
    int idx1 = cmd.indexOf(' ');
    int idx2 = cmd.indexOf(' ', idx1 + 1);
    
    if (idx1 > 0 && idx2 > 0) {
      float vol = cmd.substring(idx1 + 1, idx2).toFloat();
      float speed = cmd.substring(idx2 + 1).toFloat();
      startExtrusion(vol, speed);
    } else {
      Serial.println("✗ Format: EXTRUDE <vol1> <speed>");
    }
  }
  else if (cmd.startsWith("STATUS")) {
    printStatus();
  }
  else if (cmd.startsWith("TERMINATE") || cmd.startsWith("END")) {
    terminateSession();
  }
  else if (cmd.startsWith("STOP")) {
    emergencyHalt();
  }
  else if (cmd.startsWith("HELP")) {
    printHelp();
  }
  else {
    Serial.println("✗ Unknown command. Type HELP");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  delay(1000);
  
  Serial.println("\n\n================================");
  Serial.println("  DUAL SYRINGE CONTROLLER");
  Serial.println("  CANONICAL REFACTORED VERSION");
  Serial.println("  I2C Mode");
  Serial.println("================================");
  
  initializeMotors();
  
  Serial.println("\n✓ Ready!");
  Serial.println("Type HELP for commands\n");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }
  
  if (state.extruding) {
    checkForErrors();
  }
  
  delay(10);
}

/*
 * ========== REFACTORING NOTES FOR PHASE 7 ==========
 * 
 * CANONICAL VERSION - Single Source of Truth
 * This motor controller sketch has been refactored to:
 * 
 * 1. Use centralized configuration:
 *    - config/config.h for MOTOR1_ADDRESS, MOTOR2_ADDRESS, constants
 * 
 * 2. Use shared helper functions:
 *    - libraries/motor_controller.h for mlToSteps(), stepsToMl(), etc.
 *    - libraries/state_machine.h for system state structures
 * 
 * 3. Keep CLI logic INLINE:
 *    - homeMotors(), primeMotors(), startExtrusion() remain in-sketch
 *    - These are CLI-specific and not reused elsewhere
 *    - Abstraction would add complexity without benefit
 * 
 * 4. No code duplication with integrated firmware:
 *    - Motor helper calculations are unified in libraries
 *    - Configuration is unified in config/config.h
 *    - Each sketch focuses on its role (CLI control vs. integrated system)
 * 
 * FILES REPLACED:
 * - motor_controls.ino → firmware/motor_controls_canonical.ino
 * 
 * BUILD INSTRUCTIONS:
 * 1. Place this file in firmware/ directory
 * 2. Ensure config/ and libraries/ are in parent directory
 * 3. Compile and upload to Arduino GIGA R1 WiFi
 * 4. Open Serial Monitor at 115200 baud
 * 5. Type HELP for command list
 */
