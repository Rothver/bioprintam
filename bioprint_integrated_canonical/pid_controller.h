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
 *   #include "config/config.h"
 *   #include "libraries/thermistor_sensor.h"
 *   #include "pid_controller.h"
 *   
 *   // Global PID state
 *   PIDController heatMatPID, syringePID;
 *   
 *   void setup() {
 *     initPID(heatMatPID, KP_HEAT_MAT, SETPOINT_HEAT_MAT);
 *     initPID(syringePID, KP_SYRINGE, SETPOINT_SYRINGE);
 *   }
 *   
 *   void loop() {
 *     updateTemperatures(currentTemperatures);
 *     applyHeatControl();
 *   }
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <math.h>
#include "config.h"
#include "thermistor_sensor.h"

// ==================== PID CONTROLLER STATE STRUCT ====================
struct PIDController {
  float setpoint;          // Target temperature in °C
  float kp;                // Proportional gain
  float lastError;         // Previous error for derivative calculation (if used)
  float integral;          // Accumulated integral error (for future I control)
  float output;            // Last computed output (0-255 PWM range)
  float currentInput;      // Last measured input temperature
  bool enabled;            // Whether this controller is active
};

// ==================== GLOBAL TEMPERATURE VARIABLES ====================
// Setpoint temperatures (can be modified via UI)
float Setpoint_HeatMat = SETPOINT_HEAT_MAT;    // Default: 80°C
float Setpoint_Syringe = SETPOINT_SYRINGE;    // Default: 35°C

// Current temperatures from all 4 thermistors (set by updateTemperatures)
float currentTemperatures[NUM_THERMISTORS] = {-999, -999, -999, -999};

// Averaged temperatures for control zones
float Input_HeatMat = 25.0f;      // Average of heat mat sensors (A0, A1)
float Input_Syringe = 25.0f;      // Average of system sensors (A2, A3)
float currentDisplayTemp = 25.0f; // Display temperature (usually A2)

// PID gain parameters (configurable at compile-time via config.h)
float Kp_HeatMat = KP_HEAT_MAT;   // Proportional gain for heat mat
float Kp_Syringe = KP_SYRINGE;    // Proportional gain for syringe

// PID control outputs (0-255 PWM range)
float Output_HeatMat = 0.0f;
float Output_Syringe = 0.0f;

// Status flags
bool heatControlEnabled = false;   // Whether heating is active
bool syringesTempReached = false;  // Whether syringe system at target temperature

// ==================== INITIALIZATION ====================
/*
 * Initialize a PID controller with setpoint and gain.
 * 
 * Parameters:
 *   controller: Reference to PIDController struct to initialize
 *   kp: Proportional gain (typical: 150.0)
 *   setpoint: Target temperature in °C
 */
inline void initPID(PIDController &controller, float kp, float setpoint) {
  controller.kp = kp;
  controller.setpoint = setpoint;
  controller.output = 0.0f;
  controller.currentInput = 25.0f;
  controller.lastError = 0.0f;
  controller.integral = 0.0f;
  controller.enabled = true;
}

// ==================== GENERIC PID COMPUTATION ====================
/*
 * Compute PID output for a single control loop.
 * Currently implements P-only control. Can be extended for I and D terms.
 * 
 * Parameters:
 *   setpoint: Target temperature in °C
 *   currentValue: Measured temperature in °C
 *   deltaTime: Time step in seconds (unused for P-only, reserved for future use)
 * 
 * Returns:
 *   PID output value (0-255 for PWM applications)
 * 
 * Notes:
 * - P-only: output = Kp * error
 * - Output is clamped to 0-255 range for PWM
 * - Negative errors produce zero output (no cooling)
 */
inline float computePID(float setpoint, float currentValue, float deltaTime) {
  if (currentValue < 0 || currentValue > 150.0f) {
    return 0.0f;  // Invalid temperature reading
  }
  
  float error = setpoint - currentValue;
  float output = 150.0f * error;  // Generic Kp = 150
  output = constrain(output, 0.0f, 255.0f);
  
  return output;
}

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
  if (Input_Syringe > -999.0f && abs(Input_Syringe - SETPOINT_SYRINGE) <= TEMP_TOLERANCE) {
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
 * 
 * Control Logic:
 * - Proportional gain: error * Kp
 * - Output clamped to [0, 255] (PWM range)
 * - Control disabled entirely if heatControlEnabled = false
 * - Heat mat: Kp = KP_HEAT_MAT, setpoint = SETPOINT_HEAT_MAT (80°C)
 * - Syringe: Kp = KP_SYRINGE, setpoint = SETPOINT_SYRINGE (35°C)
 */
inline void computeDualPID() {
  // Disable outputs if control is off or sensors are reading invalid temps
  if (!heatControlEnabled || Input_HeatMat < 0 || Input_Syringe < 0) {
    Output_HeatMat = 0.0f;
    Output_Syringe = 0.0f;
    return;
  }
  
  // Compute P-only output for heat mat zone
  float error_HeatMat = SETPOINT_HEAT_MAT - Input_HeatMat;
  Output_HeatMat = KP_HEAT_MAT * error_HeatMat;
  Output_HeatMat = constrain(Output_HeatMat, 0.0f, 255.0f);
  
  // Compute P-only output for syringe zone
  float error_Syringe = SETPOINT_SYRINGE - Input_Syringe;
  Output_Syringe = KP_SYRINGE * error_Syringe;
  Output_Syringe = constrain(Output_Syringe, 0.0f, 255.0f);
}

// ==================== HEAT CONTROL APPLICATION ====================
/*
 * Apply computed PID outputs to the heating element (MOSFET).
 * Implements dual-zone control with zone priority logic.
 * 
 * Control Strategy:
 * 1. Before syringe reaches target temp: Use full heat mat output
 * 2. After syringe reaches target temp: Use min(heat mat, syringe) for stability
 * 
 * Safety:
 * - Computes fresh PID values before applying
 * - Sets PWM to 0 if control is disabled
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
  
  int finalPWM = 0;
  
  // Zone priority logic:
  // - While warming up: prioritize heat mat heating (Output_HeatMat)
  // - Once stable: limit to lower of two outputs for syringe protection
  if (!syringesTempReached) {
    finalPWM = (int)Output_HeatMat;
  } else {
    finalPWM = min((int)Output_HeatMat, (int)Output_Syringe);
  }
  
  analogWrite(MOSFET_PIN, finalPWM);
}

// ==================== STATUS CHECKING HELPERS ====================
/*
 * Check if syringe temperature is within tolerance of setpoint.
 * Used for state machine transitions and UI display updates.
 * 
 * Returns:
 *   true if syringe temperature is valid and within TEMP_TOLERANCE
 */
inline bool isSyringeTempStable() {
  return syringesTempReached;
}

/*
 * Check if heat mat temperature is valid (not sensor error).
 * 
 * Returns:
 *   true if heat mat reading is valid (> -999)
 */
inline bool isHeatMatTempValid() {
  return Input_HeatMat > -999.0f;
}

/*
 * Check if syringe temperature is valid (not sensor error).
 * 
 * Returns:
 *   true if syringe reading is valid (> -999)
 */
inline bool isSyringeTempValid() {
  return Input_Syringe > -999.0f;
}

#endif  // PID_CONTROLLER_H
