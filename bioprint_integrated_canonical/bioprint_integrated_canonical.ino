/*
 * BioPrint AM - Integrated Control System v3.1 CANONICAL
 * Arduino GIGA R1 WiFi
 * 
 * CANONICAL CONSOLIDATED VERSION
 * Merged: bioprint_integrated_v3.1_FIXED.ino and bioprint_integrated_v3.1_NO_TEMP_REQ.ino
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
 * - CONDITIONAL: #define SKIP_TEMP_VALIDATION to bypass temperature requirements
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
 * 
 * Compilation:
 *   Default: Full temperature validation required
 *   With temp skip: Arduino IDE -> Sketch -> Add #define SKIP_TEMP_VALIDATION
 *   Or compile with: -DSKIP_TEMP_VALIDATION flag
 */

// ==================== CONDITIONAL COMPILATION ====================
// Uncomment the line below to skip temperature validation
// This creates the NO_TEMP_REQ variant functionality
// #define SKIP_TEMP_VALIDATION

#include "Arduino_GigaDisplay_GFX.h"
#include "Arduino_GigaDisplayTouch.h"
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include "config.h"
#include "thermistor_sensor.h"
#include "motor_controller.h"
#include "pid_controller.h"
#include "state_machine.h"
#include "extrusion_helpers.h"
#include "ui_system.h"

// ==================== COMPILE-TIME CONFIGURATION ====================
#ifdef SKIP_TEMP_VALIDATION
  #define TEMP_VALIDATION_REQUIRED false
#else
  #define TEMP_VALIDATION_REQUIRED true
#endif

// ==================== DISPLAY & TOUCH ====================
GigaDisplay_GFX display;
Arduino_GigaDisplayTouch touchDetector;

// ==================== MOTOR OBJECTS ====================
// All motor/heat/thermistor constants are defined in config.h
TicI2C tic1;
TicI2C tic2;

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
long pendingTargetPos1 = 0;
long pendingTargetPos2 = 0;

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

// ==================== MOTOR CONTROL ====================
// initializeMotors(), syncPositionToTIC(), moveMotorsTo(), moveMotorsToWithSetSpeeds(),
// moveMotorsTimedSync(), emergencyHalt() are all defined in motor_controller.h


// Maps the current parameter page to the title and unit strings used by
// drawParameterPage() and handleParameterTouch() for display and bounds.
void getCurrentPageInfo(const char** title, const char** unit) {
  switch (currentPage) {
    case TEMPERATURE_PAGE:
      *title = "Temperature";
      *unit  = "C";
      break;
    case VOLUME1:
      *title = "Volume 1";
      *unit  = "mL";
      break;
    case VOLUME2:
      *title = "Volume 2";
      *unit  = "mL";
      break;
    case CONCENTRATION:
      *title = "Concentration";
      *unit  = "%";
      break;
    case SPEED:
      *title = "Speed";
      *unit  = "mm/s";
      break;
    default:
      *title = "Parameter";
      *unit  = "";
      break;
  }
}

// Move both motors to LOAD_POSITION (15000 steps — fully retracted for syringe loading).
// Returns true on success, false if movement fails.
bool homeMotors() {
  Serial.println("homeMotors(): moving to LOAD_POSITION (15000)");
  bool ok = moveMotorsTo(LOAD_POSITION, LOAD_POSITION, 2.0);
  if (ok) {
    arduino_pos1 = LOAD_POSITION;
    arduino_pos2 = LOAD_POSITION;
    motorsHomed  = true;
    Serial.println("homeMotors(): complete");
  } else {
    Serial.println("homeMotors(): FAILED");
  }
  return ok;
}

// Establish the full 0–15000 step range on startup.
// Moves to position 0 (minimum) then to LOAD_POSITION to confirm the
// controller's coordinate system matches the physical hardware.
void calibrateMotorsOnStartup() {
  Serial.println("calibrateMotorsOnStartup(): moving to 0");
  moveMotorsTo(0, 0, 1.0);

  delay(500);

  Serial.println("calibrateMotorsOnStartup(): moving to LOAD_POSITION (15000)");
  moveMotorsTo(LOAD_POSITION, LOAD_POSITION, 2.0);

  arduino_pos1 = LOAD_POSITION;
  arduino_pos2 = LOAD_POSITION;
  motorsHomed  = true;

  Serial.println("calibrateMotorsOnStartup(): complete");
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  delay(1000);
  
  Serial.println("\n================================");
  Serial.println("  BIOPRINT AM - INTEGRATED v3.1");
  Serial.println("  CANONICAL CONSOLIDATED VERSION");
  
  #ifdef SKIP_TEMP_VALIDATION
    Serial.println("  [NO_TEMP_REQ Variant]");
  #else
    Serial.println("  [Standard Variant]");
  #endif
  
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

  // ---- 1. Temperature update (timer-gated) ----
  static unsigned long lastTempUpdate = 0;
  if (millis() - lastTempUpdate >= TEMP_UPDATE_INTERVAL) {
    updateTemperatures();
    applyHeatControl();
    lastTempUpdate = millis();
  }

  // ---- 2. Non-blocking motor retraction to zero ----
  // Entered when user presses NO on the motor-zero-check screen.
  // Motors are commanded once, then resetCommandTimeout() is called each
  // iteration to keep the TIC alive. After RETRACTION_DURATION the loop
  // halts the motors and advances to calibration.
  static bool retractionStarted = false;
  if (currentPage == RETRACTING_TO_ZERO) {
    if (!retractionStarted) {
      tic1.clearDriverError();
      tic2.clearDriverError();
      tic1.exitSafeStart();
      tic2.exitSafeStart();
      long slowSpeed = stepsPerSecToTicUnits(mmsToStepsPerSec(0.5f));
      tic1.setMaxSpeed(slowSpeed);
      tic2.setMaxSpeed(slowSpeed);
      tic1.setTargetPosition(0);
      tic2.setTargetPosition(0);
      retractionStarted = true;
    }
    tic1.resetCommandTimeout();
    tic2.resetCommandTimeout();
    drawRetractingToZeroPage();

    if (millis() - retractionStartTime >= RETRACTION_DURATION) {
      tic1.haltAndHold();
      tic2.haltAndHold();
      arduino_pos1 = 0;
      arduino_pos2 = 0;
      retractionStarted = false;

      currentPage = CALIBRATION_IN_PROGRESS;
      drawCalibrationInProgressPage();
      delay(500);
      calibrateMotorsOnStartup();
      calibrationComplete = true;
      currentPage = WELCOME;
      drawWelcomePage();
    }
    delay(50);
    return;
  }
  retractionStarted = false;

  static MotorMoveState volumnMoveState;
  static bool volumeMoveStarted = false;
  if (currentPage == HOMING_PAGE) {
    if (!volumeMoveStarted) {
      startMotorMove(volumnMoveState, pendingTargetPos1, pendingTargetPos2, 2.0);
      volumeMoveStarted = true;
    }

    drawHomingPage();
    MotorMoveStatus status = pollMotorMove(volumnMoveState);

    if (status == ARRIVED) {
      volumeMoveStarted = false;
      arduino_pos1 = volumnMoveState.target1;
      arduino_pos2 = volumnMoveState.target2;
      currentPage = PRINT_CONFIRM;
      drawPrintConfirmPage();
    } else if (status == FAILED) {
      volumeMoveStarted = false;
      emergencyHalt();
      drawErrorPage("Motor move failed");
      currentPage = ERROR_PAGE;
    }

    delay(50);

    return;
  }

  // ---- 3. Extrusion execution (blocking while printing) ----
  // isPrinting is set by handleReadyToPrintTouch when PRINT is pressed.
  if (isPrinting && currentPage == PRINTING_PAGE) {
    bool ok = executeExtrude(config, extrusionVolume, printTime);
    isPrinting = false;
    if (ok) {
      current_state = COMPLETE;
      currentPage = POST_EXTRUSION_OPTIONS;
      drawPostExtrusionOptionsPage();
    } else {
      emergencyHalt();
      drawErrorPage("Extrusion failed");
      currentPage = ERROR_PAGE;
    }
    return;
  }

  // ---- 4. Touch input dispatch ----
  uint8_t contacts;
  GDTpoint_t points[5];
  contacts = touchDetector.getTouchPoints(points);

  if (contacts > 0) {
    int touchX = points[0].x;
    int touchY = points[0].y;

    if (!lastTouchState) {
      lastTouchState = true;

      switch (currentPage) {
        case MOTOR_ZERO_CHECK:
          handleMotorZeroCheckTouch(touchX, touchY);
          break;
        case SHUTDOWN_CONFIRM:
          handleShutdownConfirmTouch(touchX, touchY);
          break;
        case WELCOME:
          handleWelcomeTouch(touchX, touchY);
          break;
        case HOME:
          handleHomeTouch(touchX, touchY);
          break;
        case TEMPERATURE_PAGE:
          handleParameterTouch(touchX, touchY, (int*)TEMP_OPTIONS, NUM_TEMP_OPTIONS, &selectedTemp);
          break;
        case VOLUME1:
          handleParameterTouch(touchX, touchY, (int*)VOL_OPTIONS, NUM_VOL_OPTIONS, &selectedVol1);
          break;
        case VOLUME2:
          handleParameterTouch(touchX, touchY, (int*)VOL_OPTIONS, NUM_VOL_OPTIONS, &selectedVol2);
          break;
        case CONCENTRATION:
          handleParameterTouch(touchX, touchY, (int*)CONC_OPTIONS, NUM_CONC_OPTIONS, &selectedConc);
          break;
        case SPEED:
          handleParameterTouch(touchX, touchY, (int*)SPEED_OPTIONS, NUM_SPEED_OPTIONS, &selectedSpeed);
          break;
        case PRINT_CONFIRM:
          handlePrintConfirmTouch(touchX, touchY);
          break;
        case LOADING_PAGE:
          handleLoadingTouch(touchX, touchY);
          break;
        case LOADING_SYRINGES:
          handleLoadingSyringesTouch(touchX, touchY);
          break;
        case WAITING_FOR_SYRINGES:
          handleWaitingForSyringesTouch(touchX, touchY);
          break;
        case TEMP_READY:
          handleTempReadyTouch(touchX, touchY);
          break;
        case EXTRUSION_SETUP:
          handleExtrusionSetupTouch(touchX, touchY);
          break;
        case READY_TO_PRINT:
          handleReadyToPrintTouch(touchX, touchY);
          break;
        case PRINTING_PAGE:
          handlePrintingTouch(touchX, touchY);
          break;
        case POST_EXTRUSION_OPTIONS:
          handlePostExtrusionOptionsTouch(touchX, touchY);
          break;
        case PRINT_DONE:
          handlePrintDoneTouch(touchX, touchY);
          break;
        case ERROR_PAGE:
          handleErrorTouch(touchX, touchY);
          break;
        default:
          break;
      }
    }
  } else {
    lastTouchState = false;
  }

  // ---- 5. TEMP_READY: advance to systemReady after stable duration ----
  if (currentPage == TEMP_READY) {
#ifdef SKIP_TEMP_VALIDATION
    if (!systemReady && current_state == PRIMED) {
      systemReady = true;
      drawTempReadyPage();
    }
#else
    bool tempOk = (Input_Syringe > -999.0f) &&
                  (abs(Input_Syringe - Setpoint_Syringe) <= TEMP_TOLERANCE);
    bool stateOk = (current_state == PRIMED);

    if (tempOk && stateOk) {
      if (tempStableTime == 0) {
        tempStableTime = millis();
      } else if (!systemReady && (millis() - tempStableTime >= TEMP_STABLE_DURATION)) {
        systemReady = true;
        drawTempReadyPage();
      }
    } else {
      tempStableTime = 0;
      systemReady = false;
    }
#endif
  }

  // ---- 6. LOADING_PAGE: periodic redraw with live temperature values ----
  if (currentPage == LOADING_PAGE) {
    static unsigned long lastLoadingRedraw = 0;
    if (millis() - lastLoadingRedraw >= 500) {
      drawLoadingPage();
      lastLoadingRedraw = millis();
    }
  }

  delay(10);
}

/*
 * ========== NOTES FOR CONSOLIDATION ==========
 * 
 * PHASE 7 CONSOLIDATION:
 * This canonical version represents the unified firmware for both variants:
 * 
 * 1. Standard Build (Default):
 *    - Requires temperature validation
 *    - Full PID control
 *    - All safety checks enabled
 *    - Compile and upload as-is
 * 
 * 2. No-Temp-Req Build:
 *    - Uncomment #define SKIP_TEMP_VALIDATION (line ~32)
 *    - Compile and upload with that flag
 *    - Temperature checks bypassed but system remains functional
 * 
 * ALL CORE IMPLEMENTATIONS:
 * - Motor control functions fully defined in this file (inline for performance)
 * - Library calls used for: thermistor reading, PID control, UI system, state machine, extrusion helpers
 * - No duplicate code across firmware variants
 * - Single source of truth for all logic
 * 
 * DEPRECATED FILES:
 * - bioprint_integrated_v3.1_FIXED.ino (replaced by canonical version)
 * - bioprint_integrated_v3.1_NO_TEMP_REQ.ino (replaced by conditional compilation flag)
 * 
 * BUILD INSTRUCTIONS:
 * See README.md for compilation and deployment procedures
 */
