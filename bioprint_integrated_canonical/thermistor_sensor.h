/*
 * BioPrintAM Thermistor Sensor Library
 * ====================================
 * Unified Steinhart-Hart thermistor reading module for all BioPrintAM firmware variants.
 * 
 * Features:
 * - Single function interface: readThermistor(pin, sensorType)
 * - Automatic calibration lookup based on sensor type
 * - ADC resolution handling (unified to 12-bit from config.h)
 * - Steinhart-Hart implementation (no algorithm changes)
 * - Error detection and invalid reading handling
 * - Batch reading for integrated firmware (updateAllThermistors)
 * 
 * Sensor Types:
 * - HEAT_MAT_SENSOR: Beta 3435K, used on pins A0, A1
 * - SYSTEM_SENSOR: Beta 3950K, used on pins A2, A3
 * 
 * Usage:
 *   #include "config.h"
 *   #include "thermistor_sensor.h"
 *   
 *   float temp = readThermistor(A0, HEAT_MAT_SENSOR);
 *   updateAllThermistors(currentTemperatures);
 */

#ifndef THERMISTOR_SENSOR_H
#define THERMISTOR_SENSOR_H

#include <math.h>
#include "config.h"

// ==================== SENSOR TYPE ENUMERATION ====================
enum SensorType {
  HEAT_MAT_SENSOR = 0,   // Beta 3435K, series resistance 10K
  SYSTEM_SENSOR = 1      // Beta 3950K, series resistance 10K
};

// ==================== CALIBRATION LOOKUP TABLE ====================
// Internal structure to hold calibration data per sensor type
struct ThermistorCalibration {
  float seriesResistor;
  float nominalResistance;
  float nominalTemperature;
  float betaCoefficient;
};

// Returns calibration constants based on sensor type
inline ThermistorCalibration getCalibration(int sensorType) {
  ThermistorCalibration cal;
  
  if (sensorType == HEAT_MAT_SENSOR) {
    cal.seriesResistor = SERIES_RESISTOR_HEAT;
    cal.nominalResistance = THERMISTOR_NOMINAL_HEAT;
    cal.nominalTemperature = TEMPERATURE_NOMINAL_HEAT;
    cal.betaCoefficient = BETA_COEFFICIENT_HEAT;
  } else {  // SYSTEM_SENSOR
    cal.seriesResistor = SERIES_RESISTOR_SYSTEM;
    cal.nominalResistance = THERMISTOR_NOMINAL_SYSTEM;
    cal.nominalTemperature = TEMPERATURE_NOMINAL_SYSTEM;
    cal.betaCoefficient = BETA_COEFFICIENT_SYSTEM;
  }
  
  return cal;
}

// ==================== SINGLE THERMISTOR READING ====================
/*
 * Read a single thermistor and compute temperature via Steinhart-Hart equation.
 * 
 * Parameters:
 *   pinNumber: Arduino pin (A0-A3)
 *   sensorType: HEAT_MAT_SENSOR or SYSTEM_SENSOR
 * 
 * Returns:
 *   Temperature in °C, or -999.0 if reading is invalid
 * 
 * Error Handling:
 *   - ADC values near extremes (< 10 or > ADC_MAX-10) return -999.0
 *   - Invalid readings indicate sensor disconnection or short circuit
 */
inline float readThermistor(int pinNumber, int sensorType) {
  int adcValue = analogRead(pinNumber);
  
  // Reject invalid ADC readings (sensor disconnected or short)
  if (adcValue <= 10 || adcValue >= (ADC_MAX_VALUE - 10)) {
    return -999.0f;
  }
  
  // Get calibration data for this sensor type
  ThermistorCalibration cal = getCalibration(sensorType);
  
  // Convert ADC value to resistance using voltage divider equation
  // Voltage ratio = V_in / V_thermistor = ADC_MAX / ADC_value
  // For series resistor divider: 1 + R_series/R_therm = ADC_MAX / ADC_value
  // Therefore: R_therm = R_series / (ADC_MAX/ADC_value - 1)
  float voltageRatio = (float)ADC_MAX_VALUE / (float)adcValue - 1.0f;
  float resistance = cal.seriesResistor / voltageRatio;
  
  // ==================== STEINHART-HART EQUATION ====================
  // 1/T = A + B*ln(R) + C*ln(R)^3
  // Simplified form (C=0): 1/T = A + B*ln(R)
  // Using: A = 1/(T0+273.15) - (1/B)*ln(R0)
  //        B = 1/beta
  
  float steinhart = resistance / cal.nominalResistance;    // R / R0
  steinhart = log(steinhart);                              // ln(R/R0)
  steinhart /= cal.betaCoefficient;                        // * (1/beta)
  steinhart += 1.0f / (cal.nominalTemperature + 273.15f); // + (1/(T0+273.15))
  steinhart = 1.0f / steinhart;                            // invert to get T (Kelvin)
  steinhart -= 273.15f;                                    // convert to Celsius
  
  return steinhart;
}

// ==================== BATCH THERMISTOR READING (INTEGRATED FIRMWARE) ====================
/*
 * Read all 4 thermistors and store in array.
 * Used by integrated firmware to update all sensors at once.
 * 
 * Parameters:
 *   thermistorArray: Float array of size 4 to store results
 *              [0] = Heat mat sensor 1 (A0, HEAT_MAT_SENSOR)
 *              [1] = Heat mat sensor 2 (A1, HEAT_MAT_SENSOR)
 *              [2] = System sensor 1 (A2, SYSTEM_SENSOR)
 *              [3] = System sensor 2 (A3, SYSTEM_SENSOR)
 */
inline void readAllThermistors(float thermistorArray[4]) {
  thermistorArray[0] = readThermistor(THERMISTOR_PIN_A0, HEAT_MAT_SENSOR);
  thermistorArray[1] = readThermistor(THERMISTOR_PIN_A1, HEAT_MAT_SENSOR);
  thermistorArray[2] = readThermistor(THERMISTOR_PIN_A2, SYSTEM_SENSOR);
  thermistorArray[3] = readThermistor(THERMISTOR_PIN_A3, SYSTEM_SENSOR);
}

// ==================== HELPER: AVERAGE VALID READINGS ====================
/*
 * Average two thermistor readings, skipping invalid (-999.0) values.
 * Used for temperature control setpoint computation.
 * 
 * Returns:
 *   Average of valid readings, or -999.0 if no valid readings
 */
inline float averageReadings(float reading1, float reading2) {
  if (reading1 > -999.0f && reading2 > -999.0f) {
    return (reading1 + reading2) / 2.0f;
  } else if (reading1 > -999.0f) {
    return reading1;
  } else if (reading2 > -999.0f) {
    return reading2;
  } else {
    return -999.0f;
  }
}

#endif  // THERMISTOR_SENSOR_H
