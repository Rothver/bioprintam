#include <math.h>
#include "config/config.h"
#include "libraries/thermistor_sensor.h"

void setup() {
  Serial.begin(9600);
  analogReadResolution(12);
}

void loop() {
  float temps[4];

  // Read all 4 thermistors using unified library function
  readAllThermistors(temps);

  // *** Print just CSV ***
  Serial.print(temps[0], 1);
  Serial.print(",");
  Serial.print(temps[1], 1);
  Serial.print(",");
  Serial.print(temps[2], 1); 
  Serial.print(",");
  Serial.println(temps[3], 1);

  delay(1000);
}