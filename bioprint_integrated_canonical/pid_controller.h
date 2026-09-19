/*
 * BioPrintAM PID Controller Library
 * =================================
 * Temperature control and PID computation module for dual-zone heat management.
 * 
 * Features:
 * - P-only control (Kp parameter) for heat mat and syringe heating
 * - Dual-zone temperature monitoring (heat mat + syringe system)
 * - Automatic temperature stabilization detection
 * - Adaptive PWM control with zone priority logic
 * - Temperature tolerance checking for state transitions
 * 
 * Control Strategy:
 * - Heat mat (80°C): High-temperature zone for bioprinting process
 * - Syringe system (35°C): Low-temperature zone for biomaterial stability
 * - Dual PID outputs are used with min() logic for safe multi-zone control
 * 
 * Hardware:
 * - PWM output on MOSFET_PIN for heater element
 * - 4 thermistor inputs (2 heat mat, 2 system) via thermistor_sensor.h
 * 
 * Configuration:
 * - All calibration pulled from config.h
 * - Thermistor reading via thermistor_sensor.h functions
 * 
 * Usage:
 *   #include "config.h"
 *   #include "thermistor_sensor.h"
 *   #include "pid_controller.h"
 *   
 *   void loop() {
 *     updateTemperatures();
 *     applyHeatControl();   // no-op (PWM 0) unless heatControlEnabled is true
 *   }
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <math.h>
#include "config.h"
#include "thermistor_sensor.h"

// Defined in state_machine.h
extern void logFault(const char* subsystem, const char* reason);

// ==================== GLOBAL TEMPERATURE VARIABLES ====================
// Syringe setpoint (can be modified via UI). The heat mat setpoint is the
// fixed SETPOINT_HEAT_MAT constant in config.h.
float Setpoint_Syringe = SETPOINT_SYRINGE;    // Default: 35°C

// Current temperatures from all 4 thermistors (set by updateTemperatures)
float currentTemperatures[NUM_THERMISTORS] = {-999, -999, -999, -999};

// Averaged temperatures for control zones
float Input_HeatMat = 25.0f;      // Average of heat mat sensors (A0, A1)
float Input_Syringe = 25.0f;      // Average of system sensors (A2, A3)
float currentDisplayTemp = 25.0f; // Display temperature (usually A2)

// PID control outputs (0-255 PWM range)
float Output_HeatMat = 0.0f;
float Output_Syringe = 0.0f;

// Status flags
bool heatControlEnabled = false;   // Whether heating is active
bool syringesTempReached = false;  // Whether syringe system at target temperature

// ==================== TEMPERATURE UPDATE ====================
/*
 * Read all 4 thermistors and update temperature state variables.
 * Performs averaging for dual-zone control.
 * 
 * Side Effects:
 * - Updates currentTemperatures[] array
 * - Updates Input_HeatMat (average of A0, A1)
 * - Updates Input_Syringe (average of A2, A3)
 * - Updates currentDisplayTemp (usually from A2)
 * - Updates syringesTempReached flag
 * - Should be called every TEMP_UPDATE_INTERVAL (default 1000ms)
 */
inline void updateTemperatures() {
  // Read all 4 thermistors using unified library function
  readAllThermistors(currentTemperatures);
  
  // Average heat mat sensors (A0, A1)
  Input_HeatMat = averageReadings(currentTemperatures[0], currentTemperatures[1]);
  
  // Average system sensors (A2, A3) and update display temp
  Input_Syringe = averageReadings(currentTemperatures[2], currentTemperatures[3]);
  if (currentTemperatures[2] > -999.0f) {
    currentDisplayTemp = currentTemperatures[2];
  } else if (currentTemperatures[3] > -999.0f) {
    currentDisplayTemp = currentTemperatures[3];
  } else {
    currentDisplayTemp = -999.0f;
  }
  
  // Check if syringes have reached target temperature
  // Temperature is stable when within TEMP_TOLERANCE of setpoint
  if (Input_Syringe > -999.0f && abs(Input_Syringe - Setpoint_Syringe) <= TEMP_TOLERANCE) {
    syringesTempReached = true;
  } else {
    syringesTempReached = false;
  }
}

// ==================== DUAL ZONE PID CONTROL ====================
/*
 * Compute P-only PID outputs for both heat mat and syringe zones.
 * Implements dual-zone control with automatic disabling on invalid readings.
 * 
 * Side Effects:
 * - Updates Output_HeatMat (0-255 PWM range)
 * - Updates Output_Syringe (0-255 PWM range)
 * - Disables outputs if temperatures are invalid (<0 indicates sensor error)
 * - Disables outputs if the syringe exceeds Setpoint_Syringe by more than
 *   SYRINGE_OVERTEMP_MARGIN (over-temperature backstop; logs a fault once per episode)
 *
 * Control Logic:
 * - Proportional gain: error * Kp
 * - Output clamped to [0, 255] (PWM range)
 * - Control disabled entirely if heatControlEnabled = false
 * - Heat mat: Kp = KP_HEAT_MAT, setpoint = SETPOINT_HEAT_MAT (80°C)
 * - Syringe: Kp = KP_SYRINGE, setpoint = Setpoint_Syringe (user-adjustable, default 35°C)
 */
inline void computeDualPID() {
  static bool tempFaultLogged = false;
  bool tempsInvalid = (Input_HeatMat < 0 || Input_Syringe < 0);

  // Log once per fault episode, not once per loop() pass, since a
  // disconnected thermistor can hold this condition for a long time.
  if (tempsInvalid) {
    if (!tempFaultLogged) {
      logFault("temperature", "sensor reading invalid, heat output disabled");
      tempFaultLogged = true;
    }
  } else {
    tempFaultLogged = false;
  }

  // Backstop: syringe is well over its setpoint while heating is enabled.
  // Checked only while heating is enabled so a syringe that is simply cooling
  // down with heat off is not reported as a fault. Self-clears once the
  // reading drops back under the limit; logs once per episode.
  static bool overTempLogged = false;
  bool syringeOverTemp = heatControlEnabled && !tempsInvalid &&
                         (Input_Syringe > Setpoint_Syringe + SYRINGE_OVERTEMP_MARGIN);

  if (syringeOverTemp) {
    if (!overTempLogged) {
      logFault("temperature", "syringe over setpoint limit, heat output disabled");
      overTempLogged = true;
    }
  } else {
    overTempLogged = false;
  }

  // Disable outputs if control is off, sensors are invalid, or the backstop tripped
  if (!heatControlEnabled || tempsInvalid || syringeOverTemp) {
    Output_HeatMat = 0.0f;
    Output_Syringe = 0.0f;
    return;
  }
  
  // Compute P-only output for heat mat zone
  float error_HeatMat = SETPOINT_HEAT_MAT - Input_HeatMat;
  Output_HeatMat = KP_HEAT_MAT * error_HeatMat;
  Output_HeatMat = constrain(Output_HeatMat, 0.0f, 255.0f);
  
  // Compute P-only output for syringe zone
  float error_Syringe = Setpoint_Syringe - Input_Syringe;
  Output_Syringe = KP_SYRINGE * error_Syringe;
  Output_Syringe = constrain(Output_Syringe, 0.0f, 255.0f);
}

// ==================== HEAT CONTROL APPLICATION ====================
/*
 * Apply computed PID outputs to the heating element (MOSFET).
 * Implements dual-zone control with zone priority logic.
 * 
 * Control Strategy:
 * - PWM = min(heat mat output, syringe output), so the syringe zone can only
 *   ever reduce heating, and heating stops once the syringe reaches its setpoint
 *
 * Safety:
 * - Computes fresh PID values before applying
 * - Sets PWM to 0 if control is disabled, sensors are invalid, or the syringe
 *   is more than SYRINGE_OVERTEMP_MARGIN over its setpoint (see computeDualPID)
 * - Outputs on MOSFET_PIN via Arduino PWM
 * 
 * Side Effects:
 * - Calls computeDualPID() to refresh outputs
 * - Writes to MOSFET_PIN via analogWrite()
 */
inline void applyHeatControl() {
  computeDualPID();
  
  if (!heatControlEnabled) {
    analogWrite(MOSFET_PIN, 0);
    return;
  }
  
  // Always take the lower of the two zone outputs. Output_Syringe is clamped to
  // 0 whenever the syringe is at or above its setpoint, so the heater can never
  // be driven on the mat's account while the syringe is already hot.
  // While warming up, Output_Syringe saturates at 255 (any error above
  // 255 / KP_SYRINGE, ~1.7 C at KP=150), so this equals Output_HeatMat there.
  // If KP_SYRINGE is lowered, warm-up gets slower: the syringe term then limits
  // output over a wider band below the setpoint.
  int finalPWM = min((int)Output_HeatMat, (int)Output_Syringe);

  analogWrite(MOSFET_PIN, finalPWM);
}

#endif  // PID_CONTROLLER_H
