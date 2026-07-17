/*
 * BioPrintAM Configuration Header
 * ================================
 * Centralized configuration for all motor, thermistor, temperature control,
 * and UI constants across the BioPrintAM firmware ecosystem.
 * 
 * This file is the single source of truth for all calibration values,
 * avoiding duplication across integrated firmware variants and utility sketches.
 * 
 * Standardization Notes:
 * - STEPS_PER_ML: Set to 1009.0f (baseline from motor_controls.ino reference calibration)
 * - ADC Resolution: Unified to 12-bit (4095) for Arduino GIGA board
 * - Thermistor Beta: 3435 for heat mats, 3950 for system sensors
 */

#ifndef CONFIG_H
#define CONFIG_H

// ==================== MOTOR CONFIGURATION ====================
// I2C Addresses for Pololu TIC T825 stepper drivers
#define MOTOR1_ADDRESS 14
#define MOTOR2_ADDRESS 15

// Motor calibration (Actuonix P8-ST-100mm)
// Baseline from motor_controls.ino reference calibration
#define STEPS_PER_ML 1009.0f
#define MM_PER_STEP 0.006f
#define MM_PER_ML 6.05f

// Motor limits
#define MAX_VOLUME_ML 12.0f
#define MAX_STEPS ((long)(MAX_VOLUME_ML * STEPS_PER_ML))
#define LOAD_POSITION 15000L  // Loading position at top (fully retracted)

// Speed control
#define MAX_SPEED_MMS 50.0f                                      // mm/s (conservative max for hardware)
#define MAX_SPEED_STEPS_S ((long)(MAX_SPEED_MMS / MM_PER_STEP)) // steps/second
#define MAX_ACCEL 5000                                           // steps/s²

// Position tracking tolerance
#define POSITION_TOLERANCE 10

// ==================== THERMISTOR CONFIGURATION ====================
// ADC Resolution: Unified 12-bit for Arduino GIGA
#define ADC_RESOLUTION 12
#define ADC_MAX_VALUE 4095

// Physical thermistor setup
#define NUM_THERMISTORS 4
#define THERMISTOR_PIN_A0 A0
#define THERMISTOR_PIN_A1 A1
#define THERMISTOR_PIN_A2 A2
#define THERMISTOR_PIN_A3 A3

// Thermistor constants - Heat Mats (A0, A1)
// Type: NTC 10k@25C, Beta coefficient 3435K
#define SERIES_RESISTOR_HEAT 10000.0f
#define THERMISTOR_NOMINAL_HEAT 10000.0f
#define TEMPERATURE_NOMINAL_HEAT 25.0f
#define BETA_COEFFICIENT_HEAT 3435.0f

// Thermistor constants - System Sensors (A2, A3)
// Type: NTC 10k@25C, Beta coefficient 3950K
#define SERIES_RESISTOR_SYSTEM 10000.0f
#define THERMISTOR_NOMINAL_SYSTEM 10000.0f
#define TEMPERATURE_NOMINAL_SYSTEM 25.0f
#define BETA_COEFFICIENT_SYSTEM 3950.0f

// Temperature reading update interval
#define TEMP_UPDATE_INTERVAL 1000  // milliseconds

// ==================== HEAT CONTROL CONFIGURATION ====================
// PWM pin for MOSFET gate (heat control)
#define MOSFET_PIN 9

// PID/P-control parameters for heat mat
#define SETPOINT_HEAT_MAT 80.0f
#define KP_HEAT_MAT 150.0f

// P-control parameters for syringe heat
#define SETPOINT_SYRINGE 35.0f
#define KP_SYRINGE 150.0f

// Temperature stabilization criteria
#define TEMP_STABLE_DURATION 3000  // milliseconds - time temp must be stable
#define TEMP_TOLERANCE 2.0f        // °C - acceptable deviation from setpoint

// ==================== UI CONFIGURATION ====================
// Color palette (16-bit RGB565)
#define BG_COLOR 0xFFFF              // White background
#define BUTTON_COLOR 0x0010          // Dark blue
#define BUTTON_SELECTED_COLOR 0x4208 // Bright blue (selected)
#define TEXT_COLOR 0x0010            // Dark blue text
#define BUTTON_TEXT_COLOR 0xFFFF     // White text on buttons
#define CONFIRM_COLOR 0xA9CF         // Light green (confirm)
#define CANCEL_COLOR 0xF800          // Red (cancel)
#define CLEAR_COLOR 0xFD20           // Light color

// ==================== TIMING CONSTANTS ====================
// Motor zero retraction
#define RETRACTION_DURATION 15000  // milliseconds - time for full retraction to zero

// UI state machine timing
#define UI_UPDATE_INTERVAL 100  // milliseconds
#endif  // CONFIG_H
