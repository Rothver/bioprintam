#include <math.h>
#include "config/config.h"
#include "libraries/thermistor_sensor.h"

void setup() {
  Serial.begin(9600);
  analogReadResolution(12);  // Unified to 12-bit per config.h
}

void loop() {
  float temps[2];

  // Read 2 system thermistors (A0, A1) using unified library function
  // Note: Using SYSTEM_SENSOR type with 3950K beta coefficient
  temps[0] = readThermistor(THERMISTOR_PIN_A0, SYSTEM_SENSOR);
  temps[1] = readThermistor(THERMISTOR_PIN_A1, SYSTEM_SENSOR);

  // *** Print just CSV ***
  Serial.print(temps[0], 2);
  Serial.print(",");
  Serial.println(temps[1], 2);

  delay(1000);
}
